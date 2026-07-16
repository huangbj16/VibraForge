<h1 align="center">VibraForge</h1>

<p align="center">
  <strong>A Scalable Prototyping Toolkit for Creating Spatialized Vibrotactile Feedback Systems</strong>
</p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"></a>
  <img src="https://img.shields.io/badge/hardware-ESP32%20%7C%20PIC16F18313-blue.svg" alt="Hardware">
  <img src="https://img.shields.io/badge/GUI-PyQt6-green.svg" alt="GUI">
  <a href="https://forms.gle/6aN7M9MWAf4sLqWE6"><img src="https://img.shields.io/badge/request-dev%20kit-orange.svg" alt="Request a dev kit"></a>
</p>

---

VibraForge is an open-source toolkit for building **spatialized vibrotactile (haptic) feedback systems**. Small vibration actuators (LRA / VCA) are daisy-chained into scalable arrays and driven wirelessly over Bluetooth Low Energy by an ESP32-based control unit. This repository shares the complete design — mechanical, electrical, firmware, and software — so you can reproduce, modify, and integrate the toolkit into your own research and applications.

The data flow through the system is:

```
Authoring tool (GUI Editor / Unity / Python)  →  Python BLE server  →  ESP32 Control Unit (BLE)  →  chain of Vibration Units (LRA/VCA)
```

## Table of Contents

- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Running the Software](#running-the-software)
- [Hardware Notes & Debugging](#hardware-notes--debugging)
- [Getting a Dev Kit](#getting-a-dev-kit)
- [Contributing](#contributing)
- [License](#license)
- [Citation](#citation)

## Repository Structure

| Directory | Description |
|-----------|-------------|
| [Electrical_Design/](Electrical_Design/) | PCB design files (EasyEDA / Altium / Gerber) and microcontroller firmware for the control and vibration units. |
| [Mechanical_Design/](Mechanical_Design/) | 3D-printable CAD (STL / 3MF) enclosures, caps, mounts, and rings. |
| [Software_Design/](Software_Design/) | Python BLE servers and the Unity Engine integration API. |
| [GUI_Editor/](GUI_Editor/) | PyQt6 desktop app for designing, simulating, and playing haptic waveforms. |
| [manufacturing.md](manufacturing.md) | End-to-end build guide — pipeline flowchart, full BOM, PCB ordering, firmware, and assembly. |

Each subfolder contains its own `readme` with detailed, component-specific instructions.

## Getting Started

There are two ways to get the hardware, depending on whether you already have a kit:

### ▸ I have a dev kit

If you received a hardware dev kit from the authors, it ships with a printed **[Quickstart Guide](quickstart_guide.pdf)** — follow it to get up and running quickly.

### ▸ I want to build it myself

If you don't have a kit, the **[Manufacturing Guide](manufacturing.md)** walks you through the entire build end-to-end: a step-by-step pipeline flowchart, a full bill of materials, ordering assembled PCBs from JLCPCB, flashing firmware, 3D-printing the enclosures, and final assembly.

At a high level:

1. **Fabricate the boards** — order the PCBs from the Gerber files in [Electrical_Design/](Electrical_Design/) and flash the firmware (Arduino for the control unit, MPLAB X for the vibration units).
2. **Print the enclosures** — print the CAD parts in [Mechanical_Design/](Mechanical_Design/).
3. **Assemble and test** — wire the chains and run the bring-up test (below).

> Prefer to skip fabrication? You can request a ready-made kit — see [Getting a Dev Kit](#getting-a-dev-kit).

## Running the Software

Once the hardware is assembled:

1. **Bring-up test** — verify BLE control with [`Python_Test.py`](Software_Design/Python_Server/Python_Test.py). Enter vibration parameters manually on the command line. Ensure the `CHARACTERISTIC_UUID` and control unit name match the values in the control unit's Arduino firmware:

   ```bash
   python Python_Test.py -uuid CUSTOM_UUID -name CONTROL_UNIT_NAME

   # default uuid = 'f22535de-5375-44bd-8ca9-d0ea9ff9e410'
   # default name = 'QT Py ESP32-S3'
   ```

2. **Design waveforms** — once the test passes, use the [GUI Editor](GUI_Editor/) to experiment with different vibration waveforms (Sine, PWM, Triangle, Saw, etc.).

3. **Integrate with Unity** — to drive vibrations from a game engine, use the [Unity Engine API](Software_Design/Unity_Engine_API/).

For details on each program, see the [Software_Design](Software_Design/) and [GUI_Editor](GUI_Editor/) readmes.

## Hardware Notes & Debugging

### Control Unit

- **Connector orientation matters.** The order of the four chain connectors is shown in the figure below, and the USB port should face **left**. Installing a connector in the wrong direction may short the ESP32 MCU.
- If the MCU misbehaves for no clear reason, try a [factory reset](https://learn.adafruit.com/adafruit-qt-py-esp32-s3/factory-reset).

<p align="center"><img src="Figures/control_unit.png" alt="Control unit assembly and connector orientation" width="800"></p>

### Vibration Unit

- **Addressing.** Unit addresses start at **0** and are unique per chain position. Up to 16 units per chain: chain 1 → `0–15`, chain 2 → `16–31`, and so on.
- **Orientation matters.** With the programming pins on top, the **input is on the left** and the **output is on the right**. Do not reverse the order, or the driver board may be damaged by a short.
- **Status LEDs.** Each driver PCB has two LEDs: the first indicates the board is correctly powered; the second lights when the board receives a "Start" command and should be vibrating. These are useful for checking MCU status while debugging.

<p align="center"><img src="Figures/vibration_unit.png" alt="Vibration unit orientation and status LEDs" width="800"></p>

## Getting a Dev Kit

Interested in using the toolkit but would rather not build the hardware yourself? Request a development kit from the authors by filling out [this Google Form](https://forms.gle/6aN7M9MWAf4sLqWE6).

## Contributing

Contributions and bug reports are welcome! If you run into a problem or spot a bug while using the toolkit, please [open an issue](../../issues) — the authors will try to resolve it as soon as possible. Pull requests for improvements are also appreciated.

## License

This project is released under the [MIT License](LICENSE).

## Citation

If you use VibraForge in your research, we would appreciate it if you can cite our CHI 2025 paper:

```bibtex
@inproceedings{huang2025vibraforge,
  title={VibraForge: A Scalable Prototyping Toolkit For Creating Spatialized Vibrotactile Feedback Systems},
  author={Huang, Bingjian and Ren, Siyi and Luo, Yuewen and Cheng, Qilong and Cai, Hanfeng and Sang, Yeqi and Sousa, Mauricio and Dietz, Paul H and Wigdor, Daniel},
  booktitle={Proceedings of the 2025 CHI Conference on Human Factors in Computing Systems},
  pages={1--18},
  year={2025}
}
```

Have fun with the toolkit! 🎉
