# PROJECT INFO - AZURE_SEQUENCER

## Cel na teraz

Repo zawiera firmware testowe `hrdw_test` dla płytki AZURE (RP2350 / Pico 2), z prostym menu aplikacji i kalibracją DAC.

## Target firmware

- MCU: Raspberry Pi Pico 2 (RP2350)
- Build: CMake + Pico SDK
- Firmware build label: `AZURE       build 2.003`
- Binarka: `hrdw_test`

## Aplikacja `hrdw_test`

`hrdw_test` uruchamia podstawowe peryferia i ma menu startowe:

- `CALIBRATION` - kalibracja kodów DAC (kopiowana z `TRIG_GATE_SEQ`)
- `NOTES` - podglad punktow kalibracji
- `VOLTS` - podglad zakresu i wartosci napiec
- `GRIDS` - placeholder (na razie pusty)
- `TRIG SEQ` - trigger sequencer z edycją siatki
- `TR2GATE` - konwersja triggerow na gate per kanal
- `TR2ADSR` - konwersja triggerow na obwiednie AR/ASR/ADSR per kanal

W ekranach sprzętowych:

- OLED (SPI, tryb tekstowy)
- Enkodery (kwadratura + liczniki increment/decrement)
- Przyciski (`ENC_L_SW`, `ENC_R_SW`, `SW_A`) z debounce i licznikami naciśnięć
- MUX ADC (`CV_MUX`) z odczytem 4 kanałów i prezentacją napięć
- DAC MCP4728 (I2C)

### Sterowanie DAC w `hrdw_test`

- `ENC1` (lewy): wybór aktywnego kanału DAC `A/B/C/D`
- `ENC2` (prawy): zmiana napięcia wybranego kanału co `0.5V`
- Zakres napięcia zależny od aktywnego trybu kalibracji:
- `-3.0V .. +7.0V`
- `0.0V .. +10.0V`

### Sterowanie menu

- `ENC1`: wybór pozycji menu
- `ENC_R_SW`: wejście do wybranej aplikacji
- `ENC1_SW`: powrót do menu

### Kalibracja (`CALIBRATION`)

- Parametry: `CH`, `MODE`, `POINT`, `CODE`
- `MODE` przełącza zakres między `-3.0V .. +7.0V` i `0.0V .. +10.0V`
- `ENC1`: wybór parametru
- `ENC2`: zmiana wartości
- `ENC_R_SW`: zapis kalibracji do flash
- `ENC1_SW`: powrót do menu

## Struktura kodu

- `src/hal_pins_azure.h` - mapowanie GPIO dla płytki AZURE
- `src/hal_io.h`, `src/hal_io.c` - OLED, enkodery, przyciski, DAC
- `src/hal_mux_adc.h`, `src/hal_mux_adc.c` - sterowanie MUX + ADC
- `src/calibration.h`, `src/calibration.c` - kalibracja DAC + zapis do flash
- `src/hrdw_test.c` - logika aplikacji testowej

## Build

```bash
cd /Users/lazuli/Documents/PROGRAMMING/RSB_PICO/AZURE_SEQUENCER
cmake -S . -B build
cmake --build build -j4
```

Wynik:

- `build/hrdw_test.elf`
- `build/hrdw_test.uf2`
