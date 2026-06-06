# AZURE_SEQUENCER

Firmware workspace for the AZURE Eurorack module based on Raspberry Pi Pico 2 (RP2350).

Current firmware build: `2.001`

## Current Apps

- `CALIBRATION` - 4-point DAC calibration with two ranges: `-3..+7V` and `0..+10V`
- `NOTES` - calibration point summary view
- `VOLTS` - calibration range and point voltage view
- `GRIDS` - Mutable Grids-style trigger engine with parameter menu and 32-step per-channel preview
- `TRIG SEQ` - trigger sequencer with grid editing, presets and clock options
- `4XEUCLID` - 4-channel Euclidean trigger generator with per-channel steps/hits
- `TR2GATE` - 4-channel trigger-to-gate converter with per-channel source/time/probability
- `TR2ADSR` - 4-channel trigger-to-envelope generator with AR/ASR/ADSR modes

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
- `src/firmware_version.h` - AZURE firmware build label shown in the main menu
- `src/hal_mux_adc.*` - CV MUX ADC input handling
- `src/clock_input.*` - TR1 GPIO IRQ clock event queue and diagnostics
- `src/trigger_output.*` - shared us-based trigger/gate pulse scheduler
- `src/calibration.*` - calibration model + flash persistence
- `src/grids_engine.*` - Grids trigger generation core
- `src/trigseq_engine.*` - trigger sequencer core
- `src/app_settings.*` - app settings persistence
- `hardware/` - KiCad schematic and PCB

## Timing Notes

External clock on `TR1` is timestamped in a GPIO IRQ and delivered to apps through a small event queue. `TRIG SEQ`, `4XEUCLID`, `GRIDS`, and `BURST GEN` consume those events via `*_on_clock_tick(timestamp_us)` style dispatch instead of polling for the clock edge in their UI/update code.

Trigger and burst pulse widths are scheduled in microseconds by the shared trigger output engine. Timing accuracy is limited by MCP4728 I2C update latency; firmware minimizes jitter by using IRQ timestamping, cached output states and batched DAC writes. The DAC bus defaults to 400 kHz via `DAC_I2C_BAUDRATE` in `src/hal_io.c`; set it back to 100000 if a hardware build has marginal pull-ups or bus stability issues.

Pulse OFF service is driven by a 500 us repeating timer flag and serviced from the foreground timing path before OLED/UI work. The timer callback does not write I2C directly; MCP4728 writes stay out of IRQ context.

Optional 1 Hz timing diagnostics can be enabled with `ENABLE_TIMING_DIAG_PRINT` in `src/hrdw_test.c`. The diagnostics include clock IRQ count, ignored edges, queue overflow, measured BPM, max clock-event latency, max trigger-fire latency, DAC write/skip counters, and max OLED frame time.

## Hardware Timing Test Plan

1. Feed a stable master clock or FH-2 clock into `TR1`.
2. Set `TRIG SEQ` to 16 steps with hits on 1/5/9/13.
3. On an oscilloscope, measure `TR1` edge to output trigger latency, pulse width, and jitter while OLED is active and encoders are moved.
4. Test `4XEUCLID` with a stable `4/16` pattern and verify even output spacing.
5. Test `GRIDS` while changing CV/fill; CV sampling should not disturb clock-edge timing.
6. Test fast `BURST GEN` patterns; pulses should not merge or disappear except within MCP4728/I2C limits.
7. Compare DAC/I2C stability at 400 kHz versus 100 kHz if the bus shows artifacts.
