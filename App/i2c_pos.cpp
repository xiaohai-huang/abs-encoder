/**
 * @file i2c_pos.cpp
 * @brief I2C slave register map for the absolute position (see i2c_pos.h).
 *
 * The exported snapshot lives in one 32-bit word: the absolute position
 * count (turns * PositionRegister::CountsPerTurn + angle, little-endian
 * on the wire).  Around it sits one self-consistent status byte (bit0:
 * sample fresh and slew-accepted; bits 1..4: which encoder(s) failed the
 * last read) and a 16-bit sample counter (liveness for the host).  The
 * sample loop writes the snapshot with a single aligned store and the
 * transport copies it with a single aligned load at address-match time,
 * so a host read can never observe a torn position even though the
 * sample updates at 1 kHz.  After the constant full-scale count
 * (MaxPosition, register 0x07) the map carries each gear's own measured
 * angle (register 0x0B): the raw per-wheel readings next to the decoded
 * result, so a host can see what the four MT6701s actually report -- and
 * behind them, one 8-byte diagnostics block per wheel (register 0x13):
 * the cause of its last failed read, the raw chip status behind a
 * magnetic cause, saturating per-cause failure counters and the seq of
 * its last failure, so intermittent read failures can be diagnosed over
 * the same two wires.
 */
#include "i2c_pos.h"

PositionRegister positionRegister;

/* Failure counters saturate instead of wrapping: a host reading 0xFFFF
   knows it means "at least 65,535", never a silent restart at zero. */
static uint16_t SaturatingIncrement(uint16_t value)
{
    return (value == 0xFFFFu) ? value : static_cast<uint16_t>(value + 1u);
}

PositionRegister::PositionRegister()
    : _position(0u)
    , _status(0u)
    , _sampleCounter(0u)
    , _gearAngles{}
    , _diagnostics{}
    , _registerPointer(0u)
{
}

void PositionRegister::Init()
{
    _position = 0u;
    _status = 0u;
    _sampleCounter = 0u;
    for (uint8_t role = 0u; role < GearAngleCount; role++)
    {
        _gearAngles[role] = 0u;
    }
    for (uint8_t role = 0u; role < WheelCount; role++)
    {
        _diagnostics[role] = Diagnostics{0u, 0u, 0u, 0u, 0u};
    }
    _registerPointer = 0u;
}

void PositionRegister::Update(const GearPosition& position,
                              const GearAngles& angles,
                              const ReadResult (&results)[WheelCount])
{
    /* one aligned 32-bit store: a concurrent reader sees either the old or
       the new count, never a torn mix of turn and angle bytes */
    _position = static_cast<uint32_t>(position.Turns) * CountsPerTurn +
                position.Angle;
    _sampleCounter++; /* liveness: +1 per sample, wraps (docs/i2c.md) */
    const uint16_t seq = _sampleCounter; /* this sample's counter value */

    /* Each wheel's own angle, in EncoderRole order (Sun, Gear1..Gear3), so
       the raw readings sit next to the decoded position from the same
       sample. */
    const uint16_t measuredAngles[] = {angles.Sun, angles.Gear1, angles.Gear2,
                                       angles.Gear3};
    static_assert(sizeof(measuredAngles) / sizeof(measuredAngles[0]) ==
                      GearAngleCount,
                  "one measured angle per gear register, in EncoderRole order");
    for (uint8_t role = 0u; role < GearAngleCount; role++)
    {
        _gearAngles[role] = measuredAngles[role];
    }

    /* The status byte and the diagnostics block come out of the same per-
       wheel causes in this one pass, so the verdict a poller sees (health
       bits 1..4) and the detail a debugger reads (block +0) can never
       disagree.  A failure also bumps its cause's counter and stamps the
       seq of the sample it happened on. */
    uint8_t health = 0u;
    for (uint8_t role = 0u; role < WheelCount; role++)
    {
        Diagnostics&      diag   = _diagnostics[role];
        const ReadResult& result = results[role];
        diag.Cause    = result.Cause;
        diag.MgStatus = result.MgStatus;
        if (result.Cause == CauseOk)
        {
            continue;
        }
        health |= HealthBit(static_cast<EncoderRole>(role));
        if (result.Cause == CauseMagnetic)
        {
            diag.StatusFails = SaturatingIncrement(diag.StatusFails);
        }
        else
        {
            diag.FrameFails = SaturatingIncrement(diag.FrameFails);
        }
        diag.LastFailSeq = seq;
    }

    /* one byte carries both verdicts, so they can never disagree:
       the valid bit only when every read succeeded and the slew guard
       accepted, plus one bit per failed encoder read (bits 1..4) */
    _status = static_cast<uint8_t>(
        health | ((position.IsValid && health == 0u) ? StatusValid : 0u));
}

void PositionRegister::Select(uint8_t registerIndex)
{
    _registerPointer =
        (registerIndex < RegisterCount) ? registerIndex : RegisterCount;
}

/* One field of the register map: the register it starts at, how many bytes
 * wide it is, and the value it serves.  The fields *are* the map: Read()
 * walks them in order and asks each for whatever part of it falls inside
 * the host's read window, so there is no per-register branch to keep in
 * step with the layout (the static_assert in i2c_pos.h guarantees the
 * fields tile the whole map). */
struct RegisterField
{
    uint8_t  FirstRegister;
    uint8_t  ByteWidth;
    uint32_t Value;

    /* Copy the bytes of this field at or after `readStart` to
     * `destination`, returning how many were written (0 when the field is
     * entirely behind the window).  The overlap may be partial: a host is
     * allowed to start reading in the middle of a value.  The map end
     * needs no bound of its own: the fields tile exactly
     * 0x00..RegisterCount-1 (static_assert in i2c_pos.h), so none of them
     * can reach past it. */
    uint8_t Serve(uint16_t readStart, uint8_t* destination) const
    {
        uint8_t        bytesWritten = 0u;
        const uint16_t fieldEnd =
            static_cast<uint16_t>(FirstRegister + ByteWidth);
        for (uint16_t reg = FirstRegister; reg < fieldEnd; reg++)
        {
            if (reg < readStart)
            {
                continue; /* this register of mine is behind the window */
            }
            const uint8_t byteIndex =
                static_cast<uint8_t>(reg - FirstRegister); /* 0 = low byte */
            destination[bytesWritten] =
                static_cast<uint8_t>(Value >> (8u * byteIndex));
            bytesWritten++;
        }
        return bytesWritten;
    }
};

void PositionRegister::Read(uint8_t (&destination)[RegisterCount])
{
    /* One capture per transaction: every byte below is served from these
       locals, never from the live fields, so position is read as a single
       32-bit load and its four bytes can never tear even though the sample
       loop refreshes it at 1 kHz.  The gear angles are captured the same
       way, so one frame describes one sample, never a mix of two. */
    const uint32_t position = _position;
    const uint8_t  status   = _status;
    const uint16_t counter  = _sampleCounter;
    uint16_t       gearAngles[GearAngleCount];
    for (uint8_t role = 0u; role < GearAngleCount; role++)
    {
        gearAngles[role] = _gearAngles[role];
    }
    Diagnostics diagnostics[WheelCount];
    for (uint8_t role = 0u; role < WheelCount; role++)
    {
        diagnostics[role] = _diagnostics[role];
    }

    /* The window opens where the host's pointer says and always runs to the
       end of the map, because how many of these bytes the master will clock
       out is decided by the master (the shift register takes a prefix of
       what was composed).  The pointer is clamped to the map, so a start
       past the end zero-fills the whole buffer. */
    const uint16_t readStart = _registerPointer;
    uint8_t        bytesWritten = 0u;

    /* The whole map as one list of fields: the four fixed registers, then
       one gear angle per role, then five fields per diagnostics block.
       Adding a register = adding one line here.  The fields tile
       0x00..RegisterCount (static_assert in i2c_pos.h), so inside the
       window they serve exactly the bytes asked for. */
    const RegisterField fixedFields[] = {
        {0x00u, 4u, position},
        {RegisterStatus, 1u, status},
        {RegisterCounter, 2u, counter},
        {RegisterMaxPosition, 4u, MaxPosition},
    };
    static constexpr uint8_t fixedFieldCount = static_cast<uint8_t>(
        sizeof(fixedFields) / sizeof(fixedFields[0]));
    static constexpr uint8_t diagnosticsFieldCount = 5u; /* cause, mg, two
                                                            counters,
                                                            last-fail seq */

    RegisterField fields[fixedFieldCount + GearAngleCount +
                         diagnosticsFieldCount * WheelCount];
    uint8_t       fieldCount = 0u;
    for (uint8_t field = 0u; field < fixedFieldCount; field++)
    {
        fields[fieldCount++] = fixedFields[field];
    }
    for (uint8_t role = 0u; role < GearAngleCount; role++)
    {
        fields[fieldCount++] = {GearAngleRegister(static_cast<EncoderRole>(role)),
                                BytesPerGearAngle, gearAngles[role]};
    }
    for (uint8_t role = 0u; role < WheelCount; role++)
    {
        const Diagnostics& diag = diagnostics[role];
        const uint8_t reg =
            DiagnosticsRegister(static_cast<EncoderRole>(role));
        fields[fieldCount++] = {reg, 1u, diag.Cause};
        fields[fieldCount++] = {static_cast<uint8_t>(reg + 1u), 1u,
                                diag.MgStatus};
        fields[fieldCount++] = {static_cast<uint8_t>(reg + 2u), 2u,
                                diag.FrameFails};
        fields[fieldCount++] = {static_cast<uint8_t>(reg + 4u), 2u,
                                diag.StatusFails};
        fields[fieldCount++] = {static_cast<uint8_t>(reg + 6u), 2u,
                                diag.LastFailSeq};
    }

    for (uint8_t field = 0u; field < fieldCount; field++)
    {
        bytesWritten +=
            fields[field].Serve(readStart, destination + bytesWritten);
    }
    while (bytesWritten < RegisterCount)
    {
        destination[bytesWritten] = 0u; /* past the map: zero-fill, never wraps */
        bytesWritten++;
    }

    /* The read consumes the pointer: park it one past the map, so a read
       that never wrote a pointer reads zeros instead of stale map bytes. */
    _registerPointer = RegisterCount;
}
