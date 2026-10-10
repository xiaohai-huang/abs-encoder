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
 * (MaxPosition, register 0x07) the map ends with each gear's own measured
 * angle (register 0x0B): the raw per-wheel readings next to the decoded
 * result, so a host can see what the four MT6701s actually report.
 */
#include "i2c_pos.h"

PositionRegister positionRegister;

PositionRegister::PositionRegister()
    : _positionSnapshot(0u)
    , _statusByte(0u)
    , _sampleCounter(0u)
    , _gearAngle{}
    , _registerCursor(0u)
{
}

void PositionRegister::Init()
{
    _positionSnapshot = 0u;
    _statusByte = 0u;
    _sampleCounter = 0u;
    for (uint8_t role = 0u; role < GearAngleRows; role++)
    {
        _gearAngle[role] = 0u;
    }
    _registerCursor = 0u;
}

void PositionRegister::Update(const GearPosition& position,
                              const GearAngles& angles, uint8_t readHealth)
{
    /* one aligned 32-bit store: a concurrent reader sees either the old or
       the new count, never a torn mix of turn and angle bytes */
    _positionSnapshot =
        static_cast<uint32_t>(position.Turns) * CountsPerTurn + position.Angle;
    _sampleCounter++; /* liveness: +1 per sample, wraps (docs/i2c.md) */

    /* Each wheel's own angle, in EncoderRole order (Sun, Gear1..Gear3), so
       the raw readings sit next to the decoded position from the same
       sample. */
    const uint16_t measuredAngles[] = {angles.Sun, angles.Gear1, angles.Gear2,
                                       angles.Gear3};
    static_assert(sizeof(measuredAngles) / sizeof(measuredAngles[0]) ==
                      GearAngleRows,
                  "one measured angle per gear register, in EncoderRole order");
    for (uint8_t role = 0u; role < GearAngleRows; role++)
    {
        _gearAngle[role] = measuredAngles[role];
    }

    /* one byte carries both verdicts, so they can never disagree:
       the valid bit only when every read succeeded and the slew guard
       accepted, plus one bit per failed encoder read (bits 1..4) */
    _statusByte =
        static_cast<uint8_t>(readHealth |
                             ((position.IsValid && readHealth == 0u)
                                  ? StatusValid
                                  : 0u));
}

void PositionRegister::Select(uint8_t registerIndex)
{
    _registerCursor =
        (registerIndex < RegisterCount) ? registerIndex : RegisterCount;
}

/* One row of the register map: the register pointer it starts at, its
 * width in bytes, and the value it serves.  The rows *are* the map --
 * Read() walks them in ascending order and serves the bytes of the window
 * from the rows that cover it, so there is no per-register branch to keep
 * in step with the layout (the static_assert in i2c_pos.h guarantees the
 * rows tile the whole map). */
struct RegisterRow
{
    uint8_t  base;
    uint8_t  size;
    uint32_t value;
};

/* Byte `offset` of a little-endian register value. */
static uint8_t ByteOf(uint32_t value, uint8_t offset)
{
    return static_cast<uint8_t>(value >> (8u * offset));
}

/* Copy the bytes of one row that fall inside [cursor, end) to
 * `destination`, returning how many were written (0 when the window misses
 * the row). */
static uint8_t ServeRow(const RegisterRow& row, uint16_t cursor, uint16_t end,
                        uint8_t* destination)
{
    uint8_t        written = 0u;
    const uint16_t rowEnd = static_cast<uint16_t>(row.base + row.size);
    const uint16_t from = (row.base > cursor) ? row.base : cursor;
    const uint16_t to = (rowEnd < end) ? rowEnd : end;
    for (uint16_t at = from; at < to; at++)
    {
        destination[written] =
            ByteOf(row.value, static_cast<uint8_t>(at - row.base));
        written++;
    }
    return written;
}

uint8_t PositionRegister::Read(uint8_t* destination, uint8_t maxBytes)
{
    /* One capture per transaction: every byte below is served from these
       locals, never from the live fields, so position is read as a single
       32-bit load and its four bytes can never tear even though the sample
       loop refreshes it at 1 kHz.  The gear angles are captured the same
       way, so one frame describes one sample, never a mix of two. */
    const uint32_t position = _positionSnapshot;
    const uint8_t  status   = _statusByte;
    const uint16_t counter  = _sampleCounter;
    uint16_t       gearAngle[GearAngleRows];
    for (uint8_t role = 0u; role < GearAngleRows; role++)
    {
        gearAngle[role] = _gearAngle[role];
    }

    /* The bytes the host asked for; the cursor is clamped to the map, so
       everything past RegisterCount is zero-filled at the end. */
    const uint16_t cursor = _registerCursor;
    const uint16_t end = static_cast<uint16_t>(cursor + maxBytes);
    uint8_t        written = 0u;

    /* The map, in order: the four fixed registers, then one angle per role.
       Rows tile, so the bytes served in the window add up to exactly the
       bytes asked for. */
    const RegisterRow fixedRows[] = {
        {0x00u, 4u, position},
        {RegisterStatus, 1u, status},
        {RegisterCounter, 2u, counter},
        {RegisterMaxPosition, 4u, MaxPosition},
    };
    const uint8_t fixedRowCount =
        static_cast<uint8_t>(sizeof(fixedRows) / sizeof(fixedRows[0]));
    for (uint8_t row = 0u; row < fixedRowCount; row++)
    {
        written += ServeRow(fixedRows[row], cursor, end, destination + written);
    }
    for (uint8_t role = 0u; role < GearAngleRows; role++)
    {
        written += ServeRow(
            {GearAngleRegister(static_cast<EncoderRole>(role)), 2u,
             gearAngle[role]},
            cursor, end, destination + written);
    }
    while (written < maxBytes)
    {
        destination[written] = 0u; /* past the map: zero-fill, never wraps */
        written++;
    }

    /* Auto-increment, stopping at the map end. */
    const uint16_t nextCursor = static_cast<uint16_t>(cursor + written);
    _registerCursor = (nextCursor < RegisterCount)
                          ? static_cast<uint8_t>(nextCursor)
                          : RegisterCount;
    return written;
}
