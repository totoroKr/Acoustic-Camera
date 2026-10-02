# Acoustic Camera

**Four-microphone sound localisation on an STM32, with a live webcam overlay.**

EEE 416 – Microprocessor and Embedded Systems Laboratory · January 2026 · Section C1, Group 05
Department of Electrical and Electronic Engineering, BUET

Make a sound — a clap, speech, a door slam — and a marker appears on the live webcam picture at the place the sound came from.

Four INMP441 digital MEMS microphones sit at the corners of a 112 mm square, with a webcam in the centre. Sound reaches each microphone a few microseconds apart. An STM32 NUCLEO-F446RE measures those tiny time differences and turns them into a direction (azimuth and elevation). A Python program on the laptop draws that direction on the camera image.

The method is purely physics-based: no training data, no per-sound tuning. No audio is recorded or sent — only two angles leave the board

## Hardware

| Item | Qty |
|---|---|
| STM32 NUCLEO-F446RE | 1 |
| INMP441 I²S MEMS microphone module | 4 |
| USB webcam (Havit) | 1 |
| 100 kΩ resistor (data-line pull-downs) | 2 |
| 120 Ω resistor (2 in parallel = 60 Ω, WS jumper) | 2 |
| 1 µF ceramic capacitor (one per mic) | 4 |
| 10 µF capacitor (3V3 bulk) | 1 |
| Breadboard, jumper wires, 3D-printed frame | — |

### Wiring

All microphones: **VDD → 3V3**, **GND → GND**, **SCK → PB13**, **WS → PB12**.

| Mic | Position | SD pin | L/R |
|---|---|---|---|
| L1 (m0) | top-left | PC3 | GND |
| L2 (m1) | bottom-left | PC3 | 3V3 |
| R1 (m2) | top-right | PC12 | GND |
| R2 (m3) | bottom-right | PC12 | 3V3 |

Slave clock jumpers on the NUCLEO: **PB13 → PC10** and **PB12 → PA4** (through 60 Ω).
100 kΩ pull-down from **PC3 → GND** and **PC12 → GND**. 1 µF across VDD–GND at every microphone.

## Building the firmware

1. Open `acoustic_camera.ioc` in **STM32CubeMX** and click *Generate Code* (toolchain: MDK-ARM).
2. Copy `main.c` into `Core/Src/`, replacing the generated one.
3. Open `acoustic_camera.uvprojx` in **Keil µVision**.
4. Enable **CMSIS → DSP** in *Project → Manage → Run-Time Environment*.
5. In *Options for Target*: Arm Compiler 6, optimisation `-O2`, *Use MicroLIB*, *Single Precision FPU*.
6. Build (F7) and flash (F8).

Key CubeMX settings: HCLK 180 MHz, PLLI2S = 96 MHz (M = 8, N = 192, R = 2), I²S2 master RX / I²S3 slave RX, Philips, 16-bit on 32-bit frame, 32 kHz, DMA circular, **PB12/PB13 GPIO speed = Very High**, USART2 115200 8N1.

## Running the overlay

```bash
pip install numpy opencv-python pyserial
python sound_overlay.py --port COM4 --hfov 62
```

- Replace `COM4` with your board's port (`/dev/ttyACM0` on Linux).
- `--hfov` is your webcam's horizontal field of view in degrees.
- **Close PuTTY first** — only one program can use the COM port.

**Overlay keys:** `m` mirror · `t` trail · `c` clear · `f` freeze · `s` save PNG · `q` quit

## Serial output and commands

Open the port in PuTTY at **115200 baud** to see what the board reports:

```
QUIET   lvl   112   bg   190   needs   380      ← no sound
WEAK    q 1.42 < 1.80   lvl 401 (x2.1)          ← sound, but not trusted
* AZ  +12.4   EL  -3.1   q 3.42   lvl 287       ← a measurement
ANGLE az=12.40 el=-3.10                         ← line read by the overlay
```

| Key | Action |
|---|---|
| `+` / `-` | less / more sensitive |
| `q` / `Q` | raise / lower the quality threshold |
| `z` / `Z` | set current direction as zero / clear |
| `v` | verbose (per-pair lags) |
| `c` | detailed pair-quality dump |
| `h` | show / hide QUIET lines |
| `x` | force I²S re-sync |

## Known limitations

- **Azimuth is less reliable than elevation.** The left–right pairs combine one microphone from each I²S peripheral, and the slave's clock travels over jumper wires, so they are not perfectly synchronised. The top–bottom pairs stay within one peripheral and work well.
- A flat array cannot tell front from back.
- Pure tones (beeps) are ambiguous — use broadband sounds such as claps or speech.
- One dominant sound source at a time.

## Team

| Name | Student ID | Contribution |
|---|---|---|
| Md. Tahir Hassan | 2106139 | CubeMX / clock configuration, documentation |
| K.M Shihab Rahmatullah | 2106140 | Circuit assembly and design, array build and wiring |
| Jarif Shahriar Ahmed | 2106141 | I²S + DMA bring-up, GCC-PHAT firmware |
| Shahriar Sadik | 2106142 | Debugging and measurement |
| Maisha Mahfuza | 2106143 | 3D prototype design, array build, wiring and debugging |

## References

- C. H. Knapp and G. C. Carter, "The generalized correlation method for estimation of time delay," *IEEE Trans. ASSP*, 1976.
- InvenSense, *INMP441 datasheet*.
- STMicroelectronics, *RM0390 STM32F446 reference manual*.
- Arm, *CMSIS-DSP library*.
