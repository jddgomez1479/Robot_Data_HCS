# STM32 Blackpill: MTi-1 IMU & MD49 Motor Driver Firmware

High-performance, non-blocking firmware developed for **STM32 (Blackpill)** integrating an **Xsens MTi-1 IMU** (running at 100 Hz with quaternions) and a dual **MD49 Motor Driver** via UART, streaming unified high-speed telemetry over **USB-CDC** to **ROS2 jazzy**.

---

## Key Features

* **Multi-Timer Architecture:** Optimized main loop using `HAL_GetTick()` to decouple high-priority tasks from secondary background polls.
* **Stable 100 Hz IMU:** Continuous reading and parsing of XBus packets (Accelerometer, Gyroscope, and Quaternions) without blocking.
* **Asynchronous Motor Management:** 
  * Encoder velocity polling every **45 ms**.
  * General state reading and safety checks (`Safety_Check`) every **100 ms**.
  * Bidirectional speed control via incoming serial commands.
* **Unified USB-CDC Telemetry:** Streaming a single formatted string optimized to minimize CPU overhead, achieving an effective frequency in ROS2 of **95–98 Hz**.

---

## Hardware Stack

* **Microcontroller:** STM32 Blackpill (STM32F401 / STM32F411)
* **IMU:** Xsens MTi-1 (configured in XBus output mode)
* **Motor Driver:** MD49 (UART communication at 38400 baud)
* **Connectivity:** Native USB (Virtual COM Port / CDC) to PC/SBC (ROS2)

---

## Data Frame Format (USB-CDC)

The microcontroller outputs a structured string terminated with `\r\n`:


`"E,enc1,enc2;V,volt;I,i1,i2;S,err,mod,ac;A,ax,ay,az;G,gx,gy,gz;O,q0,q1,q2,q3;F,freq\r\n"`


## Field Descriptions:

* E: Encoders (Encoder 1, Encoder 2)
* V: Battery voltageI: Motor currents (Motor 1, Motor 2)
* S: MD49 status (t_err, mod, ac)
* A: IMU Accelerometer (a_x, a_y, a_z in m/s^2)
* G: IMU Gyroscope (g_x, g_y, g_z in rad/s)
* O: Orientation in Quaternions (q_0, q_1, q_2, q_3)
* F: Real-time measured loop frequency (Hz)
