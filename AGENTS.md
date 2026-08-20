# AGENTS.md

Firmware for a battery-free multi-turn absolute encoder on an STM32F103C8Tx (Cortex-M3, 72 MHz from 8 MHz HSE × PLL9). Active peripherals: SPI1, SPI2, I2C1, TIM2, GPIO — four MT6701 absolute encoders (User Labels CSN1..CSN4), two per SPI bus. Multi-turn position is decoded from coprime gear phases (`docs/architecture.md`), so the firmware keeps no persistent state — the four MT6701s are the only source of truth. TIM2 (1 kHz update interrupt) is the sample clock; the DWT cycle counter is the µs clock. I2C1 is the host port: the device acts as an I2C slave (address 0x50) serving the position register map (`docs/i2c.md`).

## Layout

- `Core/` — application code (CubeMX-generated `main.c`, `spi.c`, `i2c.c`, `tim.c`, `gpio.c`, IT/MSP files)
- `App/` — hand-written application logic, split by hardware dependency:
  - `hal.h` — the only header the logic includes; a `const app_hal_t` struct of function pointers (SPI byte exchange + CS, µs clock, busy-wait delay) — grblHAL-style HAL
- `mt6701.c/.h` — MT6701 SSI protocol layer (24-bit frame decode, status validation, retries; CRC-6 validation, X^6+X+1 per datasheet §7.8.2 — see `docs/MT6701.md`; toggle with `MT6701_CRC6_ENABLED`). All calls are indexed by `encoder_role_t` — `ENC_SUN` is the input shaft, `ENC_GEAR_1..3` the driven gears (wiring: ENC_GEAR_1/2 on SPI1 via CSN1/CSN2, ENC_SUN/ENC_GEAR_3 on SPI2 via CSN4/CSN3 — the `s_enc` table in `hal_stm32.c`)
- `gear_config.h` — compile-time mechanical config: input + driven tooth counts, turn-field width, slew limit; changing gears = edit this header, rebuild, reflash. Also defines the `encoder_role_t` enum (`ENC_SUN`, `ENC_GEAR_1..3`) that names every encoder everywhere. The CSN/bus wiring per role is the role-keyed `s_enc` table in `hal_stm32.c`
- `gear_decode.c/.h` — absolute multi-turn decode from coprime gear phases: per-sample CRT over the three gear residues, slew-guarded (jumps > `GEAR_MAX_TURNS_DELTA` rejected as misreads, last position held). No storage; the only state is the last accepted turn count
- `i2c_pos.c/.h` — I2C slave host port: the decoded position as a 5-register map (turns + fine angle + status/valid), pure logic (no HAL). The snapshot is packed into one 32-bit word so an address-match read can never tear; wire protocol in `docs/i2c.md`
- `hal_stm32.c` — STM32 backend (SPI1 + SPI2, role-keyed `s_enc` wiring table binding each `encoder_role_t` to its CSN pin and SPI bus, DWT µs clock, and the I2C1 slave transport: own address applied at runtime, EV/ER interrupts, listen-IT callbacks); the only firmware file that touches STM32 HAL
- `sim/` — PC build of the same `App/` logic (test only, not in the firmware build):
  - `hal_sim.c` — PC backend (QPC clock, SPI bridged to the fake chip)
  - `mt6701_slave_sim.c/.h` — simulated MT6701 speaking the real 24-bit SSI frame, with fault/CRC injection controls
  - `sim_main.c` — assert-based test harness, `Makefile` (`make run`)
- `Drivers/` — STM32F1xx HAL + CMSIS (vendor code, don't hand-edit)
- `multi-turn-absolute-encoder.ioc` — STM32CubeMX project; source of truth for peripheral config
- `docs/MT6701.md` — MT6701 datasheet summary (Rev 1.8); the protocol layer is verified against it
- `docs/architecture.md` — battery-free multi-turn design (coprime gear phase tracking: 13-tooth input gear driving 17/19/23-tooth gears, each with its own MT6701; 7,429-turn absolute range)
- `docs/i2c.md` — I2C host port spec (slave address, register map, transactions)
- `STM32F103XX_FLASH.ld` — linker script (custom scatter file referenced by `.eide/eide.yml`)
- `startup_stm32f103xb.s` — startup assembly
- `.eide/eide.yml` — Embedded IDE (EIDE) project: Debug and Release targets, toolchain and flash settings
- `CMakeLists.txt`, `CMakePresets.json`, `cmake/` — CubeMX-generated CMake byproduct; unused (EIDE is the build system actually used) and gitignored — CubeMX recreates them on every code generation

## Build

Builds go through the EIDE extension — from the VS Code UI (`Ctrl+Shift+B`, tasks in `.vscode/tasks.json`) or, for agents, through EIDE's MCP server:

- The MCP server runs from the extension when `EIDE.MCP.Server.Enable` is on (port 8940; both keys are set in `awesome-abs-encoder.code-workspace` → `settings`). It only lives while VS Code is open with the project — it auto-exits ~3 s after the extension host disconnects.
- Register it in the agent client as HTTP at `http://localhost:8940/mcp` (the server entry needs `"noProxy": "localhost,127.0.0.1"` to bypass the system proxy). Tools take the project `uid` from `.eide/eide.yml` (`miscInfo.uid`).
- Prefer MCP tools over hand-editing `.eide/eide.yml`; if you do edit it manually, call `eide_reload` first so the extension regenerates `builder.params` (the snapshot `unify_builder` consumes — CLI builds with stale params silently ignore config changes).

Build outputs (`.elf`/`.hex`/`.map`) land in `build/<Config>/`; `build/` is gitignored. Flashing is via STLink SWD (cortex-debug) from within VS Code — there is no CLI flash script.

Toolchain settings (from `.eide/eide.yml`): C11, `-Wall`, newlib-nano, `-lm`, function/data sections, `--gc-sections`; Debug = `-Og`, Release = `-Os`; output is ELF + HEX (no `.bin`).

### PC test build (sim/)

`App/` logic (mt6701 + gear_decode + i2c_pos) is compiled against the sim backend and the fake MT6701 chip — same source, no firmware needed:

```
cd sim && make run        # MinGW gcc; WinLibs installed via winget lives in
                          # %LOCALAPPDATA%\Microsoft\WinGet\Packages\...\mingw64\bin
```

Exit code 0 = all checks pass. CRC-6 validation is on by default; to test the no-CRC path: `make run CFLAGS="-std=c11 -Wall -Wextra -g -O0 -I../App -DMT6701_CRC6_ENABLED=0"`.

## CubeMX regeneration rules

Code under `Core/` is regenerated from the `.ioc`. Any manual code must live inside `/* USER CODE BEGIN x */ ... /* USER CODE END x */` blocks — anything outside them is overwritten on regeneration. New user source files, include paths, and defines go in `.eide/eide.yml` (and the CMake lists if you also keep that path working); `srcDirs` covers `Core/`, `Drivers/` and `App/`. Don't put hand code in `Core/` outside USER CODE blocks — put it in `App/`. Keep `sim/` out of the firmware build (it's not in `srcDirs`); files there are PC-only.

## Gotchas

- The four CSN pins (User Labels CSN1..CSN4) are configured as push-pull outputs initialized **HIGH** (deselected) via `PinState` in the `.ioc` (high speed); `app_hal.init()` also raises them defensively before the first frame. Keep referencing them by label — CubeMX regenerates `CSNx_Pin`/`CSNx_GPIO_Port` from the labels.
- The I2C slave address is applied at runtime: generated `i2c.c` keeps `OwnAddress1 = 0`, `hal_stm32.c` re-inits with `I2C_POS_ADDR` in `app_hal.init()`. The `I2C1_EV/ER_IRQHandler` stubs live in the file-level USER CODE block of `stm32f1xx_it.c` (the `.ioc` has no I2C1 NVIC lines) — if you enable those lines in CubeMX, delete the stubs. PB6/PB7 are open-drain; the bus needs external pull-ups.
- The CMake files are a CubeMX byproduct (gitignored and untracked; CubeMX recreates them on every code generation). If they're ever run: CMake presets write to `build/<preset>/` — `cmake --preset Debug` would clobber EIDE's `build/Debug` artifacts, so avoid it.
- clangd (`.clangd`) reads `compile_commands.json` from `build/Debug`; if IntelliSense is stale, run a Debug build first.
- `.clang-format` is Microsoft-based: 4 spaces, Linux brace style, `SortIncludes: false`, no column limit. Match it for new code.
- Commit messages follow Conventional Commits: `type(scope): description`, e.g. `fix(encoder): correct SPI read timing` or `feat(core): add raw mode`. One line, imperative mood.
