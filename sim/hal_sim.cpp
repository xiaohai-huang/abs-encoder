/**
 * @file hal_sim.cpp
 * @brief PC backend: SPI bridged to the simulated MT6701 chips (one per
 *        encoder role), QPC clock.  Drop-in replacement for hal_stm32.cpp
 *        so the logic runs unmodified on the host.
 */
#include "hal.h"

#include "mt6701_slave_sim.h"

#include <windows.h>

class SimHal : public Hal
{
public:
    void Init() override;
    uint8_t SpiTransfer(EncoderRole encoder, uint8_t txByte) override;
    void SelectChip(EncoderRole encoder, bool asserted) override;
    uint32_t GetMicroseconds() override;
    void DelayMicroseconds(uint32_t microseconds) override;

private:
    static LARGE_INTEGER _performanceFrequency;
};

LARGE_INTEGER SimHal::_performanceFrequency;

void SimHal::Init()
{
    QueryPerformanceFrequency(&_performanceFrequency);
    Mt6701SlaveSim::Init();
}

uint8_t SimHal::SpiTransfer(EncoderRole encoder, uint8_t txByte)
{
    return Mt6701SlaveSim::Transfer(encoder, txByte);
}

void SimHal::SelectChip(EncoderRole encoder, bool asserted)
{
    Mt6701SlaveSim::SelectChip(encoder, asserted);
}

uint32_t SimHal::GetMicroseconds()
{
    LARGE_INTEGER counter;
    QueryPerformanceCounter(&counter);
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(counter.QuadPart) * 1000000ull) /
        _performanceFrequency.QuadPart);
}

void SimHal::DelayMicroseconds(uint32_t microseconds)
{
    uint32_t deadline = GetMicroseconds() + microseconds;
    while ((int32_t)(GetMicroseconds() - deadline) < 0)
    {
    }
}

static SimHal _simHal;
Hal& hal = _simHal;