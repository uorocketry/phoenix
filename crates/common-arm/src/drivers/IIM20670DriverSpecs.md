# IIM20670 Driver Specifications

This document outlines the necessary specifications extracted from the datasheet (DS-000183 Rev 1.0) and reference driver implementation for developing a Rust driver for the IIM-20670 6‑axis IMU sensor.

## 1. General Information

* **Device:** IIM-20670 SmartIndustrial™ 6‑axis MotionTracking® MEMS Device
* **Key Features:**

  * 3‑axis gyroscope (±41 dps…±1966 dps) with 16‑bit ADC
  * 3‑axis accelerometer (±2 g…±65 g) with 16‑bit ADC, high‑/low‑resolution modes
  * Dual temperature sensors
  * Programmable digital filters per axis
  * CRC‑protected SPI communication
* **Package:** 24‑pin DQFN (4.5 × 4.5 × 1.1 mm)
* **Operating Temperature:** –40 °C…+105 °C
* **Shock Tolerance:** 10 000 g

## 2. Electrical Characteristics (Driver‑Relevant)

| Parameter             | Min   | Typ     | Max   | Notes                  |
| --------------------- | ----- | ------- | ----- | ---------------------- |
| VDD                   | 3.0 V | —       | 5.5 V | Supply voltage         |
| VDDIO                 | 3.0 V | —       | 5.5 V | I/O supply voltage     |
| Supply Current        | —     | < 10 mA | —     | All operating modes    |
| Peak Supply Current   | —     | —       | —     | During ADC conversions |
| GPIO Drive Capability | —     | —       | —     | Standard CMOS          |

## 3. Communication Interface

### 3.1 SPI

* **Mode:** SPI Mode 0 (CPOL=0, CPHA=0)
* **Max Clock:** 10 MHz
* **Frame Format:** 32‑bit (8‑bit command + 16‑bit data + 8‑bit CRC)
* **Data Order:** MSB first
* **Pins:**

  * **SCLK** – Serial clock
  * **MOSI** – Master Out, Slave In
  * **MISO** – Master In, Slave Out
  * **NCS**  – Chip Select (active low)

### 3.2 Bank‑Based Register Access

* **Bank 0:** Sensor data & filter registers
* **Banks 1–7:** Configuration registers (unlock with sequence 0x0002→0x0001→0x0004 to MODE register)
* **Bank Switch Delay:** ≥100 µs after writing BANK\_SELECT register

## 4. Register Commands

| Name                | Bank | Address | Description                          |
| ------------------- | ---- | ------- | ------------------------------------ |
| GYRO\_X\_DATA       | 0    | 0x00    | Raw gyroscope X-axis (16 bits)       |
| GYRO\_Y\_DATA       | 0    | 0x01    | Raw gyroscope Y-axis                 |
| GYRO\_Z\_DATA       | 0    | 0x02    | Raw gyroscope Z-axis                 |
| TEMP1\_DATA         | 0    | 0x03    | Temperature sensor 1                 |
| ACCEL\_X\_DATA      | 0    | 0x04    | Raw accelerometer X-axis             |
| ACCEL\_Y\_DATA      | 0    | 0x05    | Raw accelerometer Y-axis             |
| ACCEL\_Z\_DATA      | 0    | 0x06    | Raw accelerometer Z-axis             |
| TEMP2\_DATA         | 0    | 0x07    | Temperature sensor 2                 |
| ACCEL\_X\_DATA\_LR  | 0    | 0x08    | Low-resolution accel X-axis          |
| ACCEL\_Y\_DATA\_LR  | 0    | 0x09    | Low-resolution accel Y-axis          |
| ACCEL\_Z\_DATA\_LR  | 0    | 0x0A    | Low-resolution accel Z-axis          |
| FIXED\_VALUE        | 0    | 0x0B    | Read-back constant (0xAA55 expected) |
| FILTER\_Y\_Z        | 0    | 0x0C    | Y/Z axis filter configuration        |
| FILTER\_X           | 0    | 0x0E    | X-axis filter configuration          |
| TEMP12\_DELTA       | 0    | 0x0F    | Temperature difference (Temp1–Temp2) |
| RESET\_CONTROL      | 0    | 0x18    | Soft/hard reset commands             |
| MODE                | 0    | 0x19    | Mode control and lock bits           |
| BANK\_SELECT        | —    | 0x1F    | Bank selection                       |
| EN\_ACCEL\_SELFTEST | 7    | 0x11    | Enable accelerometer self-test       |
| EN\_GYRO\_SELFTEST  | 7    | 0x12    | Enable gyroscope self-test           |
| ACCEL\_FS\_SEL      | 6    | 0x14    | Accelerometer full-scale selection   |
| GYRO\_FS\_SEL       | 7    | 0x15    | Gyroscope full-scale selection       |
| SELF\_TEST          | 7    | 0x16    | Self-test status                     |

## 5. Filter Configuration

Nine cutoff options: 10, 12.5, 27, 30, 46, 60, 250, 300, 400 Hz.  See datasheet Table 14–16 and driver `encode_filter_bits_x` / `encode_filter_bits_yz` implementations.

## 6. Full‑Scale Selection

| Sensor        | Bank | Reg            | Bits   | Range Options (LSB/unit)          |
| ------------- | ---- | -------------- | ------ | --------------------------------- |
| Gyroscope     | 7    | GYRO\_FS\_SEL  | \[3:0] | ±41…±1966 dps (800…16.67 LSB/dps) |
| Accelerometer | 6    | ACCEL\_FS\_SEL | \[2:0] | ±2…±65 g (16000…500 LSB/g)        |

* Defaults: ±655 dps, ±16 g

## 7. CRC & Status

* **CRC‑8**: polynomial 0x1D, init 0xFF, final = bitwise NOT (§5.2)
* **Status bits**: RS1\:RS0 = bits 1–0 of first MISO byte

  * `01` = OK, `10` = in progress, `00/11` = error (Table 13)

## 8. Initialization & Integrity Check

1. Power-on → NCS high → delay ≥10 ms
2. Soft reset (`RESET_CONTROL = 0x0002`) → delay ≥200 ms
3. (Optional hard reset) → delay ≥3 ms
4. Verify `WHO_AM_I` = 0xF3; `FIXED_VALUE` = 0xAA55
5. Configure full-scale & filters → lock writes (MODE bit)

## 9. Data Read Sequence

```text
set_bank(0)
gx = read_register(0x00)
gy = read_register(0x01)
gz = read_register(0x02)
temp = read_register(0x03)
ax = read_register(0x04)
ay = read_register(0x05)
az = read_register(0x06)
```

* Convert raw → units:

  * Gyro (dps)  = raw / sensitivity
  * Accel (g)  = raw / sensitivity
  * Temp (°C)  = 25.0 + raw/20.0
  * ΔT (°C)    = raw\_delta/20.0

## 10. Timing Requirements

| Operation          | Delay   |
| ------------------ | ------- |
| Bank switch        | ≥100 µs |
| Soft reset → ready | ≥200 ms |
| Hard reset → ready | ≥3 ms   |

## 11. Pin Summary

| Pin  | Name | Type | Function                 |
| ---- | ---- | ---- | ------------------------ |
| 1    | VDD  | PWR  | 3.0–5.5 V supply         |
| 2    | GND  | PWR  | Ground                   |
| 3    | SCLK | I/O  | SPI clock                |
| 4    | MOSI | I    | SPI data in              |
| 5    | MISO | O    | SPI data out             |
| 6    | NCS  | I    | Chip select (active low) |
| 7–24 | NC   | —    | No connect               |

## 12. Hardware Notes

* Place 100 nF decoupling capacitor close to VDD→GND
* Keep NCS high between transfers to avoid CRC errors
* Use self-test bits and check `SELF_TEST` status for diagnostics

## 13. References

* DS‑000183 Rev 1.0 — IIM‑20670 Datasheet

  * Section 4: Data conversion formulas
  * Section 5.2 & Table 13: CRC & status bits
  * Section 6.6–6.8: Sensor data & filters
  * Section 6.12–6.17: Reset, mode, FS selection
  * Table 14–16: Digital filter descriptions
  * Table 17–18: Full-scale settings

*Prepared based on driver implementation and IIM-20670 datasheet.*