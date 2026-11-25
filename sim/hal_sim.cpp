/**
 * @file hal_sim.cpp
 * @brief PC backend: SPI bridged to the simulated MT6701 chips (one per
 *        encoder role), monotonic host clock.  Drop-in replacement for
 *        hal_stm32.cpp so the logic runs unmodified on the host.
 */
#include "hal.h"

#include "mt6701_slave_sim.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <chrono>
#endif

class SimHal : public Hal
{
public:
    void Init() override;
    uint8_t SpiTransfer(EncoderRole encoder, uint8_t txByte) override;
    void SelectChip(EncoderRole encoder, bool asserted) override;
    uint32_t GetMicroseconds() override;
    void DelayMicroseconds(uint32_t microseconds) override;

private:
#ifdef _WIN32
    static LARGE_INTEGER _performanceFrequency;
#else
    std::chrono::steady_clock::time_point _startTime;
#endif
};

#ifdef _WIN32
LARGE_INTEGER SimHal::_performanceFrequency;
#endif

void SimHal::Init()
{
#ifdef _WIN32
    QueryPerformanceFrequency(&_performanceFrequency);
#else
    _startTime = std::chrono::steady_clock::now();
#endif
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
#ifdef _WIN32
    LARGE_INTEGER counter;
    QueryPerformanceCounter(&counter);
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(counter.QuadPart) * 1000000ull) /
        _performanceFrequency.QuadPart);
#else
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - _startTime)
            .count());
#endif
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