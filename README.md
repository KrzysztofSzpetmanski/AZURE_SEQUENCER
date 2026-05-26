# AZURE_SEQUENCER

Firmware workspace for the AZURE Eurorack module based on Raspberry Pi Pico 2 (RP2350).

## Current Apps

- `HRDW_TEST` - hardware I/O diagnostics, DAC/CV view, encoder/button tests
- `CALIBRATION` - 4-point DAC calibration (`-3V`, `0V`, `+3V`, `+6V`) with flash persistence
- `GRIDS` - Mutable Grids-style trigger engine with parameter menu and 32-step per-channel preview
- `TRIGSEQ` - trigger sequencer with grid editing and clock options
- `4XEUCLID` - 4-channel Euclidean trigger generator with per-channel steps/hits

## Build

```bash
cd /Users/lazuli/Documents/PROGRAMMING/RSB_PICO/AZURE_SEQUENCER
cmake -S . -B build
cmake --build build -j4
```

Artifacts:

- `build/hrdw_test.elf`
- `build/hrdw_test.uf2`

## Flash

Standard Pico UF2 drag-and-drop flow:

1. Hold BOOTSEL while connecting Pico 2 via USB.
2. Copy `build/hrdw_test.uf2` to the mounted RPI-RP2 drive.

## Project Structure

- `src/hrdw_test.c` - app loop, UI, app state machines
- `src/hal_io.*` - OLED, encoders, buttons, DAC I/O
- `src/hal_mux_adc.*` - CV MUX ADC input handling
- `src/calibration.*` - calibration model + flash persistence
- `src/grids_engine.*` - Grids trigger generation core
- `src/trigseq_engine.*` - trigger sequencer core
- `src/app_settings.*` - app settings persistence
- `hardware/` - KiCad schematic and PCB
