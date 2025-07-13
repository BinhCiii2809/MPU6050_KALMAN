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
You should see a table displaying your GPIO status and the WiringPi version.

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

Navigate to `Interface Options` → `I2C` → Select `Enable`.




