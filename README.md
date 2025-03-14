# TVC Flight Computer 

## 1. Project Overview
### 1.1 Purpose & Objectives
- Design a **Thrust Vector Control (TVC) Flight Computer** based on **STM32WB55 & Jetson Nano**.
- Integrate **IMU, LiDAR, Barometer, and GPS** for advanced **Guidance, Navigation, and Control (GNC)**.
- Secure telemetry and communication using **AES + Post-Quantum Hybrid Encryption**.
- Ensure **robust flight stability and precision landing** using real-time sensor fusion.

### 1.2 System Architecture
- **Main MCU:** STM32WB55 (Handles real-time TVC control & BLE telemetry)
- **AI Processing Unit:** Jetson Nano (Processes LiDAR & advanced navigation)
- **Sensors:**
  - RPLIDAR M1 → Terrain mapping & obstacle detection
  - MPU6050 → Attitude estimation (Pitch, Roll, Yaw)
  - BMP280 → Altitude sensing
  - GPS Module (Optional) → Position tracking
- **Power Supply:** LiPo 3S 11.1V + DC-DC Converters
- **Actuators:** Servo Motors for TVC Control (MG995 / DS3225)
- **Communication:** BLE, UART, SPI (Jetson ↔ STM32WB55)

---

## 2. Hardware Documentation
### 2.1 Bill of Materials (BOM)
Refer to [BOM.xlsx](BOM.xlsx) for the complete component list.

### 2.2 Schematic & PCB Layout
- **Schematic:** `TVC_Flight_Computer_Schematic.pdf`
- **PCB Layout:** `TVC_PCB_Design.kicad_pcb`

### 2.3 Pin Mapping
| Peripheral       | STM32WB55 Pin  | Jetson Nano GPIO |
|-----------------|---------------|------------------|
| IMU (MPU6050)   | I2C (SCL, SDA) | N/A              |
| Barometer (BMP280) | I2C (SCL, SDA) | N/A              |
| LiDAR (RPLIDAR M1) | UART RX/TX    | UART2 TX/RX      |
| TVC Servo 1     | PWM Output    | N/A              |
| TVC Servo 2     | PWM Output    | N/A              |
| LoRa (Optional) | SPI (MISO/MOSI) | N/A              |

---

## 3. Firmware Documentation
### 3.1 System Code Architecture
- **State Machine:** Implements **Launch → Ascent → TVC Control → Descent → Landing**
- **Interrupt Handling:** Uses **TIMx PWM & External Interrupts for fast TVC response**

### 3.2 Communication Protocols
- **Jetson ↔ STM32WB55:** UART @ 115200 baud
- **BLE Telemetry:** Sends real-time **sensor & flight data** to ground station
- **Encryption:** AES-256 + Post-Quantum Hybrid Security

### 3.3 Deployment Guide
#### **STM32WB55 Firmware (Using STM32CubeIDE)**
1. Clone repo: `git clone https://github.com/your_repo.git`
2. Open STM32CubeIDE and import project
3. Build and flash firmware via ST-Link

#### **Jetson Nano AI Processing (Python)**
1. Install dependencies:  
   ```bash
   sudo apt update && sudo apt install python3-pip
   pip install numpy scipy matplotlib
   ```
2. Run LiDAR SLAM: `python3 lidar_processing.py`

---

## 4. TVC Algorithm & GNC System
### 4.1 TVC Control Algorithm (PID)
- Uses **PID Control** to stabilize thrust vectoring.
- Adjusts TVC servos based on **IMU + LiDAR terrain feedback**.

### 4.2 Sensor Fusion (Kalman Filter)
- **Combines IMU, Barometer & LiDAR** to estimate altitude & velocity.
- Implements **adaptive filtering** for better noise reduction.

---

## 5. Testing & Validation
### 5.1 Unit Testing
- IMU, Barometer, and LiDAR sensor validation using test scripts.

### 5.2 Flight Testing
- **Pre-launch checklist** → Sensor calibration, power checks, telemetry validation.
- **Post-flight analysis** → Log processing, trajectory correction.

---

## 6. Version Control & Repository Structure
```
/TVC_Flight_Computer_Project
│-- /firmware_stm32/          # STM32WB55 Firmware (C, FreeRTOS)
│-- /ai_processing/           # Jetson Nano LiDAR Processing (Python)
│-- /hardware_design/         # KiCad Schematics & PCB
│-- /docs/                    # Documentation & Research Papers
│-- /test_scripts/            # Unit Test Scripts
│-- README.md                 # Main Documentation
```

---

## 7. Future Upgrades & Improvements
✅ **LoRa Long-Range Telemetry**  
✅ **Advanced AI-based Navigation (SLAM + Path Planning)**  
✅ **Real-Time Data Logging & Cloud Dashboard**  

---

## 8. Contributors
- **Your Name** – Embedded Systems Engineer  
- **Contributors** – Open for collaboration!  

---

## 9. License
This project is licensed under **MIT License**.
