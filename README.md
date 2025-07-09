# Four‑Axis Vacuum Stage for Advanced Nano‑Manufacturing

A compact 4‑degree‑of‑freedom robotic vacuum stage built for uniform 3D nanocoating of irregular objects inside a magnetron sputtering chamber. This system integrates mechanical, electrical, and software components to deliver precise, high‑vacuum motion control and real‑time user interaction.

---

## Table of Contents

1. [Features](#features)  
2. [Hardware Design](#hardware-design)  
3. [Control & Software](#control--software)  

   


---

## Features

- **4‑DOF Vacuum‑Compatible Arm**: Aluminum‑extrusion structure, screw‑drive joint, electromagnetic brake for safe, power‑loss protection.  
- **High‑Precision Motion**: Positional repeatability ≤ 1 mm over 100 cycles under vacuum and high temperature.  
- **Uniform 3D Nanocoating**: Demonstrated conformal chromium sputtering on both flat and curved substrates.  
- **Modular Control System**: STM32F407 MCU, RS‑485 bus, Emm42 stepper drivers, and a 7″ TFT touchscreen HMI.  
- **Robust Electronics**: Custom PCB with optical isolation, 24 V power distribution, and noise suppression.

---

## Hardware Design

1. **Mechanical**  
   - 4 axes: base rotation, arm pitch, wrist tilt, substrate turntable  
   - Vacuum‑rated bearings, V‑groove rails, precision lead screws  
2. **Electrical**  
   - STM32F407 control board  
   - RS‑485 communication to four Emm42 stepper drivers  
   - Custom power & signal PCB with optical couplers  
3. **Vacuum Integration**  
   - Compact footprint for insertion into standard sputtering chambers  
   - Viewports for in‑situ monitoring  

---

## Control & Software

- **Firmware**:  
  - Written in C using STM32CubeIDE  
  - Motion profiles and closed‑loop status polling  
- **Communication**:  
  - RS‑485 protocol for deterministic stepper control  
  - UART for HMI updates  
- **User Interface**:  
  - Touchscreen menus for manual jog, preset routines, and emergency stop

## Demo Video
<video controls width="720" preload="metadata">
  <source src="demo.mp4" type="video/mp4">
  您的浏览器不支持 HTML5 video 标签。
</video>

