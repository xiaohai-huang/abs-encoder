/**
 * @file interactive_main.cpp
 * @brief Interactive console for the PC simulation: move the modelled
 *        input shaft, read the fake MT6701s over the real SSI path, run the
 *        firmware sample loop and inspect the I2C register map.
 *
 * Build and run from sim/:  make interactive   (or ./build/enc-shell)
 */
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "app_entry.h"
#include "gear_config.h"
#include "hal.h"
#include "i2c_pos.h"
#include "mt6701.h"
#include "mt6701_slave_sim.h"

namespace
{
const char* RoleName(EncoderRole role)
{
    switch (role)
    {
    case EncoderRole::Sun:
        return "Sun";
    case EncoderRole::Gear1:
        return "Gear1";
    case EncoderRole::Gear2:
        return "Gear2";
    default:
        return "Gear3";
    }
}

bool ParseRole(const char* text, EncoderRole& role)
{
    if (std::strcmp(text, "sun") == 0)
    {
        role = EncoderRole::Sun;
        return true;
    }
    if (std::strcmp(text, "g1") == 0 || std::strcmp(text, "gear1") == 0)
    {
        role = EncoderRole::Gear1;
        return true;
    }
    if (std::strcmp(text, "g2") == 0 || std::strcmp(text, "gear2") == 0)
    {
        role = EncoderRole::Gear2;
        return true;
    }
    if (std::strcmp(text, "g3") == 0 || std::strcmp(text, "gear3") == 0)
    {
        role = EncoderRole::Gear3;
        return true;
    }
    return false;
}

const char* CauseName(uint8_t cause)
{
    switch (cause)
    {
    case PositionRegister::CauseOk:
        return "ok";
    case PositionRegister::CauseMagnetic:
        return "magnetic";
    case PositionRegister::CauseNoFrame:
        return "no-frame";
    default:
        return "?";
    }
}

const EncoderRole Roles[] = {EncoderRole::Sun, EncoderRole::Gear1,
                             EncoderRole::Gear2, EncoderRole::Gear3};

uint16_t ReadLe16(const uint8_t* bytes, unsigned index)
{
    return static_cast<uint16_t>(bytes[index] |
                                 (static_cast<uint16_t>(bytes[index + 1u])
                                  << 8u));
}

uint32_t ReadLe32(const uint8_t* bytes, unsigned index)
{
    return static_cast<uint32_t>(bytes[index]) |
           (static_cast<uint32_t>(bytes[index + 1u]) << 8u) |
           (static_cast<uint32_t>(bytes[index + 2u]) << 16u) |
           (static_cast<uint32_t>(bytes[index + 3u]) << 24u);
}

/* Read the four chips the way the firmware does and print what came back. */
void ReadAll()
{
    for (size_t roleIndex = 0u;
         roleIndex < sizeof(Roles) / sizeof(Roles[0]); roleIndex++)
    {
        Mt6701Sample sample;
        const int result = Mt6701::ReadSample(hal, Roles[roleIndex], sample);
        if (result == Mt6701::Ok)
        {
            std::printf("  %-5s angle=%5u  status=0x%02X\n",
                        RoleName(Roles[roleIndex]), sample.Angle,
                        sample.Status);
        }
        else
        {
            std::printf("  %-5s READ FAILED (%d)  status=0x%02X\n",
                        RoleName(Roles[roleIndex]), result, sample.Status);
        }
    }
}

void PrintShaft()
{
    const uint32_t shaft = Mt6701SlaveSim::ShaftPosition();
    std::printf("shaft: %u turns + %5u/16384  (count %u)\n", shaft / 16384u,
                shaft % 16384u, shaft);
}

/* One firmware sample: four SSI reads -> decode -> publish to the map. */
void RunSample()
{
    AppProcessSample();

    uint8_t bytes[PositionRegister::RegisterCount];
    positionRegister.Select(0u);
    positionRegister.Read(bytes);

    const uint32_t count = ReadLe32(bytes, 0u);
    const uint8_t status = bytes[PositionRegister::RegisterStatus];
    std::printf("position: %u turns + %5u/16384  valid=%d  status=0x%02X\n",
                count / 16384u, count % 16384u,
                (status & PositionRegister::StatusValid) != 0u ? 1 : 0,
                status);
}

/* Dump the register map the host would read over I2C. */
void DumpMap()
{
    uint8_t bytes[PositionRegister::RegisterCount];
    positionRegister.Select(0u);
    positionRegister.Read(bytes);

    const uint32_t count = ReadLe32(bytes, 0u);
    const uint8_t status = bytes[PositionRegister::RegisterStatus];
    std::printf("I2C map (%u bytes)\n", PositionRegister::RegisterCount);
    std::printf("  0x00 count   : %u  (%u turns + %u/16384)\n", count,
                count / 16384u, count % 16384u);
    std::printf("  0x04 status  : 0x%02X  valid=%d\n", status,
                (status & PositionRegister::StatusValid) != 0u ? 1 : 0);
    std::printf("  0x05 counter : %u\n",
                ReadLe16(bytes, PositionRegister::RegisterStatus + 1u));

    for (size_t roleIndex = 0u;
         roleIndex < sizeof(Roles) / sizeof(Roles[0]); roleIndex++)
    {
        const unsigned angleReg =
            PositionRegister::GearAngleRegister(Roles[roleIndex]);
        std::printf("  0x%02X %-5s angle: %u\n", angleReg,
                    RoleName(Roles[roleIndex]),
                    ReadLe16(bytes, angleReg));
    }

    for (size_t roleIndex = 0u;
         roleIndex < sizeof(Roles) / sizeof(Roles[0]); roleIndex++)
    {
        const unsigned diagReg =
            PositionRegister::DiagnosticsRegister(Roles[roleIndex]);
        std::printf(
            "  0x%02X %-5s diag : cause=%s mg=0x%02X frameFails=%u "
            "statusFails=%u lastFailSeq=%u\n",
            diagReg, RoleName(Roles[roleIndex]),
            CauseName(bytes[diagReg]), bytes[diagReg + 1u],
            ReadLe16(bytes, diagReg + 2u), ReadLe16(bytes, diagReg + 4u),
            ReadLe16(bytes, diagReg + 6u));
    }
}

void ApplyFault(const char* roleText, const char* kind, const char* valueText)
{
    EncoderRole role;
    if (!ParseRole(roleText, role))
    {
        std::printf("unknown role '%s' (sun, g1, g2, g3)\n", roleText);
        return;
    }

    const bool enabled = std::strcmp(valueText, "off") != 0;
    if (std::strcmp(kind, "track") == 0 ||
        std::strcmp(kind, "trackloss") == 0)
    {
        Mt6701SlaveSim::SetTrackLoss(role, enabled);
        std::printf("%s track loss %s\n", RoleName(role),
                    enabled ? "on" : "off");
    }
    else if (std::strcmp(kind, "crc") == 0)
    {
        Mt6701SlaveSim::SetCrcBroken(role, enabled);
        std::printf("%s CRC corruption %s\n", RoleName(role),
                    enabled ? "on" : "off");
    }
    else if (std::strcmp(kind, "stuck") == 0)
    {
        Mt6701SlaveSim::SetStuck(role, enabled);
        std::printf("%s stuck output %s\n", RoleName(role),
                    enabled ? "on" : "off");
    }
    else if (std::strcmp(kind, "field") == 0)
    {
        const unsigned status = static_cast<unsigned>(std::atoi(valueText));
        Mt6701SlaveSim::SetFieldStatus(role,
                                       static_cast<uint8_t>(status & 0x03u));
        std::printf("%s field status %u\n", RoleName(role), status & 0x03u);
    }
    else
    {
        std::printf("unknown fault '%s' (track, field, crc, stuck)\n", kind);
    }
}

void PrintHelp()
{
    std::printf(
        "commands:\n"
        "  pos <turns> [sun]        place the shaft (sun = 0..16383 counts)\n"
        "  move <counts>            move the shaft by signed counts\n"
        "  shaft                    show the modelled shaft position\n"
        "  read                     read the four chips over SSI\n"
        "  sample                   run AppProcessSample (reads + decode + map)\n"
        "  map                      dump the I2C register map\n"
        "  fault <role> <kind> [v]  role: sun|g1|g2|g3\n"
        "                           kind: track|field|crc|stuck (on/off)\n"
        "  help                     this text\n"
        "  quit                     leave\n");
}
} // namespace

int main()
{
    AppInit();
    std::printf("encoder simulation console.  Type 'help' for commands.\n");

    std::string line;
    for (;;)
    {
        std::printf("> ");
        std::fflush(stdout);
        if (!std::getline(std::cin, line))
        {
            break;
        }
        if (line.empty())
        {
            continue;
        }

        if (line == "quit" || line == "exit" || line == "q")
        {
            break;
        }
        if (line == "help" || line == "?")
        {
            PrintHelp();
        }
        else if (line == "shaft")
        {
            PrintShaft();
        }
        else if (line == "read")
        {
            ReadAll();
        }
        else if (line == "sample")
        {
            RunSample();
        }
        else if (line == "map")
        {
            DumpMap();
        }
        else if (line.compare(0u, 4u, "pos ") == 0)
        {
            unsigned turns = 0u;
            unsigned sun = 0u;
            const int fields =
                std::sscanf(line.c_str() + 4, "%u %u", &turns, &sun);
            if (fields < 1 || turns >= GearConfig::TurnRange || sun >= 16384u)
            {
                std::printf("usage: pos <turns 0..%u> [sun 0..16383]\n",
                            GearConfig::TurnRange - 1u);
            }
            else
            {
                Mt6701SlaveSim::SetShaftPosition(turns * 16384u + sun);
                PrintShaft();
            }
        }
        else if (line.compare(0u, 5u, "move ") == 0)
        {
            int delta = 0;
            if (std::sscanf(line.c_str() + 5, "%d", &delta) != 1)
            {
                std::printf("usage: move <counts>\n");
            }
            else
            {
                Mt6701SlaveSim::AdvanceShaft(static_cast<int32_t>(delta));
                PrintShaft();
            }
        }
        else if (line.compare(0u, 6u, "fault ") == 0)
        {
            char roleText[16] = {0};
            char kind[16] = {0};
            char valueText[16] = {0};
            std::sscanf(line.c_str(), "fault %15s %15s %15s", roleText, kind,
                        valueText);
            ApplyFault(roleText, kind, valueText);
        }
        else
        {
            std::printf("unknown command '%s'; type 'help'\n", line.c_str());
        }
    }

    return 0;
}
