# AGENTS.md

Firmware for multi-turn absolute encoders on an STM32F103C8Tx (Cortex-M3, 72 MHz from 8 MHz HSE × PLL9). Active peripherals: SPI1, SPI2, I2C1, GPIO — four MT6701 absolute encoders (User Labels CSN1..CSN4), two per SPI bus.

## Layout

- `Core/` — application code (CubeMX-generated `main.c`, `spi.c`, `i2c.c`, `gpio.c`, IT/MSP files)
- `App/` — hand-written application logic, split by hardware dependency:
  - `hal.h` — the only header the logic includes; a `const app_hal_t` struct of function pointers (SPI byte exchange + CS, µs clock, delay, NVS) — grblHAL-style HAL
- `mt6701.c/.h` — MT6701 SSI protocol layer (24-bit frame decode, status validation, retries; CRC-6 validation, X^6+X+1 per datasheet §7.8.2 — see `docs/MT6701.md`; toggle with `MT6701_CRC6_ENABLED`). All calls are indexed by encoder 0..3 — encoders 0/1 share SPI1 (CSN1/CSN2), 2/3 share SPI2 (CSN3/CSN4)
- `multi_turn.c/.h` — multi-turn accumulation with NVS persistence (one slot per encoder)
- `hal_stm32.c` — STM32 backend (SPI1 + SPI2, four encoders via CSN1..CSN4, DWT µs clock, last flash page as NVS with page-preserving writes); the only firmware file that touches STM32 HAL
- `sim/` — PC build of the same `App/` logic (test only, not in the firmware build):
  - `hal_sim.c` — PC backend (QPC clock, `nvs.bin` file, SPI bridged to the fake chip)
  - `mt6701_slave_sim.c/.h` — simulated MT6701 speaking the real 24-bit SSI frame, with fault/CRC injection controls
  - `sim_main.c` — assert-based test harness, `Makefile` (`make run`)
- `Drivers/` — STM32F1xx HAL + CMSIS (vendor code, don't hand-edit)
- `multi-turn-absolute-encoder.ioc` — STM32CubeMX project; source of truth for peripheral config
- `docs/MT6701.md` — MT6701 datasheet summary (Rev 1.8); the protocol layer is verified against it
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

`App/` logic (mt6701 + multi_turn) is compiled against the sim backend and the fake MT6701 chip — same source, no firmware needed:

```
cd sim && make run        # MinGW gcc; WinLibs installed via winget lives in
                          # %LOCALAPPDATA%\Microsoft\WinGet\Packages\...\mingw64\bin
```

`make run` deletes `sim/nvs.bin` first; the harness also removes it at startup, so stale state can't leak between runs. Exit code 0 = all checks pass. CRC-6 validation is on by default; to test the no-CRC path: `make run CFLAGS="-std=c11 -Wall -Wextra -g -O0 -I../App -DMT6701_CRC6_ENABLED=0"`.

## CubeMX regeneration rules

Code under `Core/` is regenerated from the `.ioc`. Any manual code must live inside `/* USER CODE BEGIN x */ ... /* USER CODE END x */` blocks — anything outside them is overwritten on regeneration. New user source files, include paths, and defines go in `.eide/eide.yml` (and the CMake lists if you also keep that path working); `srcDirs` covers `Core/`, `Drivers/` and `App/`. Don't put hand code in `Core/` outside USER CODE blocks — put it in `App/`. Keep `sim/` out of the firmware build (it's not in `srcDirs`); files there are PC-only.

## Gotchas

- The four CSN pins (User Labels CSN1..CSN4) are configured as push-pull outputs initialized **HIGH** (deselected) via `PinState` in the `.ioc` (high speed); `app_hal.init()` also raises them defensively before the first frame. Keep referencing them by label — CubeMX regenerates `CSNx_Pin`/`CSNx_GPIO_Port` from the labels.
- The CMake files are a CubeMX byproduct (gitignored and untracked; CubeMX recreates them on every code generation). If they're ever run: CMake presets write to `build/<preset>/` — `cmake --preset Debug` would clobber EIDE's `build/Debug` artifacts, so avoid it.
- clangd (`.clangd`) reads `compile_commands.json` from `build/Debug`; if IntelliSense is stale, run a Debug build first.
- `.clang-format` is Microsoft-based: 4 spaces, Linux brace style, `SortIncludes: false`, no column limit. Match it for new code.
- Commit messages follow Conventional Commits: `type(scope): description`, e.g. `fix(encoder): correct SPI read timing` or `feat(core): add raw mode`. One line, imperative mood.
