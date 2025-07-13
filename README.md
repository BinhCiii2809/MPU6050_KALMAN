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

### ⚠️ I2C Address & Connection Check

Before using this library, make sure your MPU6050 is properly connected via I2C to the Raspberry Pi.

#### 🔍 How to Check I2C Connection:
Open a terminal on your Raspberry Pi and run:

```bash
$gpio i2cdetect
```
> 📌 This command will scan and display all I2C addresses connected to your Pi.
### ✅ Expected Result:
- If your MPU6050 is detected, you will see an address like `0x68` (default).
- If the displayed address is not `0x68`, you must update the library:
### 🛠 Update I2C Address in Code:
1. Open the file `mpu6050.h`
2. Locate this line:
```c
#define MPU_ADDRESS 0x68
```
3. Change `0x68` to the actual address shown in the terminal (e.g.,` 0x69`)
4. Save the file
5. Recompile your code
### ❌ No Address Detected?
If no address appears in the terminal (i.e., the scan output is empty or just dashes --):
- Check your wiring (VCC, GND, SDA, SCL)
- Ensure I2C is enabled (raspi-config)
- Confirm the sensor is powered properly
> #### ⚠️ If the device is still not detected, the connection is likely faulty or the sensor is damaged.
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
**Download all files** into the same directory on your Raspberry Pi.
---
 
## 📁 Files
- `Makefile`
---

1. Open a `terminal raspberry` and navigate to the directory containing the files.
2. Run the following commands:

```bash
$make
$make install
```
3. Once installed, execute the main program:

```bash
$./mpu6050
```
#### 📌 Uninstall and delete all file installed 
```bash
$make uninstall
$make clean
```
### ⚙️ Raspberry Pi I²C Setup
 
- Important: Make sure I²C is enabled on your Raspberry Pi. If not, follow these steps:
```bash
$raspi-config
```

> Navigate to `Interface Options` → `I2C` → Select `Enable`.

---

## 📚 MPU6050 Library

This is a lightweight, modular C library for interfacing with the **MPU6050** sensor using a configurable `typedef struct` for setup. It supports accelerometer, gyroscope, angle estimation (via simple integration), and sampling configuration.
---

## 📁 Files

- `mpu6050.h` – Header file containing function prototypes and macros.
- `mpu6050.c` – Source file implementing the functions.
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

#### 🔹 `int16_t read_raw(uint8_t reg);`
Reads raw 16-bit data from the specified MPU6050 register.
> 📌 This function first reads the high byte (8 bits) at reg, then reads the low byte (8 bits) from reg + 1.
> The two bytes are combined to return a signed 16-bit integer.
---

#### 🔹 `float mpu6050_read_accel(uint8_t reg)`;
Reads and converts raw **accelerometer** data from the specified register into acceleration in **g** (gravitational units).

#### 🔧 Parameters:
- reg: The starting register address for the desired accelerometer axis (e.g., MPU6050_ACCEL_XOUT_H)

#### 📐 Returns:
- float: Acceleration in units of **g** (1g ≈ 9.81 m/s²)

#### 📘 Details:
- Internally reads 16-bit raw data: high byte first, then low byte
- Converts using current MPU6050_ACCEL_CONFIG scale (±2g, ±4g, ±8g, ±16g)
- Useful for estimating **static tilt angles** when the device is not moving
- The raw value is divided by a **scale sensitivity factor** based on the configured full-scale range
---

#### 🔹 `float mpu6050_read_gyro(uint8_t reg)`;
Reads and converts raw **gyroscope** data from the specified register into **angular velocity** in degrees per second (°/s).

#### 🔧 Parameters:
- `reg`: The starting register address for the desired gyroscope axis (e.g., `MPU6050_GYRO_XOUT_H`)

#### 📐 Returns:
- `float`: Angular velocity in units of **degrees per second (°/s)**

#### 📘 Details:
- Internally reads 16-bit raw data: high byte first, then low byte
- Converts using the current `MPU6050_GYRO_CONFIG` scale (±250, ±500, ±1000, ±2000 °/s)
- The raw value is divided by a **scale sensitivity factor** based on the configured full-scale range
---

#### 🔹 `uint64_t millis();`
Returns the number of milliseconds elapsed since the  system boot.

---

## 📚 Kalman Filter Module for MPU6050
Implements **1D Kalman Filter** designed for sensor fusion of accelerometer and gyroscope data from the **MPU6050**.  
It provides accurate, stable angle estimation (e.g., **pitch** and **roll**) by minimizing noise and drift.
---

### 📌 Why Use Accelerometer as Measurement and Gyroscope as Rate?

#### ✅ Gyroscope (Gyro)
 - Pro: Very responsive and smooth for short-term angular rate changes.
 - Con: Suffers from drift over time due to bias accumulation.

#### ✅ Accelerometer (Accel)
 - Pro: Provides absolute angle estimation based on gravity direction.
 - Con: Noisy and sensitive (shaking, movement).
---

### 🧠 Strategy
> 🔧 Prediction (Time Update)
> Use gyroscope angular rate to integrate angle over time:
```c
angle += (gyroRate - bias) * dt;
```

> 🔧 Correction (Measurement Update)
> Use accelerometer-derived angle to correct drifted estimate:
```c
angle += K * (accAngle - angle);
```

---

## 📁 Files

- `KALMAN_FILTER.h` – Header file containing function prototypes and macros.
- `KALMAN_FILTER.c` – Source file implementing the functions.
---

## ⚙️ Kalman Filter Structure

```c
typedef struct {
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement

    float angle; // Ouput- part of the 2x1 state vector
    float bias; // Ouput- gyro bias- part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix
} Kalman_t;
```

| Function                                                                       | Description                                             |
| ------------------------------------------------------------------------------ | ----------------------------------------------------- |
| `void kalman_init(Kalman_t *k);`                                               | Initialize Kalman filter with default tuning parameters |
| `float kalman_getAngle(Kalman_t *k, float newAngle, float newRate, float dt);` | Update the filter and return the estimated angle      |
| `void kalman_setAngle(Kalman_t *k, float angle);`                              | Reset the estimated angle                             |
| `float kalman_getRate(Kalman_t *k);`                                           | Get the filtered rate                                 |
| `void kalman_setQangle(Kalman_t *k, float Q_angle);`                           | Tuning                                                |
| `void kalman_setQbias(Kalman_t *k, float Q_bias);`                             | Tuning                                                |
| `void kalman_setRmeasure(Kalman_t *k, float R_measure);`                       | Tuning                                                |
