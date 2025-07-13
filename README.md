## 📚 Reference Documentation

For more detailed technical specifications, refer to the official datasheet:

🔗 **MPU6050 Datasheet**  
[View Datasheet (PDF)](https://www.alldatasheet.com/datasheet-pdf/view/1132809/TDK/MPU6050.html)

---

## 🚀 Feature of MPU6050

- Read accelerometer and gyro data
- Configure sensitivity ranges
- Calibrate sensor offsets
- Interrupt support

---

## 📊 Specifications

| Feature                    | Details                                  |
|----------------------------|------------------------------------------|
| Gyroscope Range            | ±250, ±500, ±1000, ±2000 °/s             |
| Accelerometer Range        | ±2g, ±4g, ±8g, ±16g                      |
| Communication Protocol     | I²C                                      |
| Digital Interface          | 400kHz I²C                               |
| MPU6050 Address            | 0x68                                     |
| Operating Voltage          | 3.3V                                     |
| Sensor Resolution          | 16-bit ADC                               |
| Built-in Temperature Sensor| Yes                                      |

---

## 🧰 Building and Running on Raspberry Pi

## 🔌 Installing WiringPi on Raspberry Pi

This project may rely on WiringPi to access handle I²C communication. If WiringPi is not already installed on your Raspberry Pi, follow these steps:

1. Clone the maintained version of WiringPi:

```bash
$git clone https://github.com/WiringPi/WiringPi.git
$cd WiringPi
```

2. Build and install the library:

```bash
$./build
```

3. Verify installation:

```bash
$gpio -v
$gpio readall
```
> You should see a table displaying your GPIO status and the WiringPi version.

#### 💡 If these commands fail, you may need to run them with `sudo` or ensure your system is up to date.

```bash
$sudo apt-get update
$sudo apt-get upgrade
```

### 🧱 To compile and run this library on Raspberry Pi:

1. **Download all files** into the same directory on your Raspberry Pi.
2. Open a terminal and navigate to the directory containing the files.
3. Run the following commands:

```bash
$make
$make install
```

4. Once installed, execute the main program:

```bash
$./mpu6050
```

### ⚙️ Raspberry Pi I²C Setup
 
- Important: Make sure I²C is enabled on your Raspberry Pi. If not, follow these steps:
```bash
$raspi-config
```

> Navigate to `Interface Options` → `I2C` → Select `Enable`.

## 🧭 MPU6050 Library

This is a lightweight, modular C library for interfacing with the **MPU6050** sensor using a configurable `typedef struct` for setup. It supports accelerometer, gyroscope, angle estimation (via simple integration), and sampling configuration.

---

### 📦 `MPU6050_CONFIG_t` Structure

The `MPU6050_CONFIG_t` structure is used to configure the MPU6050 during initialization. Below is the breakdown:

```c
typedef struct {
    // --- Power Management ---
    uint8_t enable_temp;          // Enable temperature sensor: 1 = ON, 0 = OFF
    uint8_t sleep_mode;           // Sleep mode: 1 = SLEEP, 0 = WAKE
    MPU6050_CLKSEL_t clock_sel;   // Clock source selection

    // --- Interrupt Configuration ---
    uint8_t data_ready_interrupt;     // Interrupt on new data ready (bit 0)
    uint8_t fifo_overflow_interrupt;  // Interrupt on FIFO overflow (bit 6)
    uint8_t i2c_master_interrupt;     // Interrupt on I2C master events (bit 5)

    // --- Configuration Register ---
    MPU6050_ExtSync_t ext_sync;   // External frame sync (bits 5-3)
    MPU6050_DLPF_t dlpf_cfg;      // Digital Low Pass Filter (bits 2-0)

    // --- Sensor Sensitivity Configuration ---
    MPU6050_ACCEL_CONFIG accel_config;  // Accelerometer full-scale range
    MPU6050_GYRO_CONFIG gyro_config;    // Gyroscope full-scale range

} MPU6050_CONFIG_t;
```
---
#### 🔹 `int mpu6050_init(const MPU6050_CONFIG_t *config);`
Initializes the MPU6050 device with the given configuration.
#### 🔧 Parameters:
- `config`: A pointer to a user-defined `MPU6050_CONFIG_t` struct that contains all initial setup values.
---

#### 🔹 `void mpu6050_set_sample_time(uint8_t samptime_div);`
Sets the sample rate divider (GOR=8kHz when DLPF OFF, =1KHz when DLPF ON`(see reg26)`, effective = GOR / (1 + samptime_div)).
