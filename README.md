# Technical Documentation

## LiDAR Connector Pinout

![LiDAR Pinout](https://github.com/pav2000/LidarStm32f103/blob/main/lidar.jpg?raw=true)

*Image credit: [pav2000/LidarStm32f103](https://github.com/pav2000/LidarStm32f103)*

---

## Table of Contents

1. [Hardware Overview](#hardware-overview)
2. [LiDAR Sensor: MB-1R2T V1.5.8](#lidar-sensor-mb-1r2t-v158)
3. [USB-to-Serial Converter: FT232RL](#usb-to-serial-converter-ft232rl)
4. [Physical Connection & Wiring](#physical-connection--wiring)
5. [Serial Communication Protocol](#serial-communication-protocol)
6. [Data Packet Structure](#data-packet-structure)
7. [Byte-Level Protocol Analysis](#byte-level-protocol-analysis)
8. [Distance Measurement Principles](#distance-measurement-principles)
9. [Signal Quality & Filtering](#signal-quality--filtering)
10. [Timing & Data Rates](#timing--data-rates)
11. [Software Implementation](#software-implementation)
12. [Troubleshooting](#troubleshooting)

---

## Hardware Overview

This project interfaces a **Chinese MB-1R2T V1.5.8** 2D LiDAR scanner with a computer via an **FTDI FT232RL** USB-to-Serial converter. The system enables real-time 360° environmental scanning with distance measurements up to 12 meters.

### System Block Diagram

```
┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐
│                 │      │                 │      │                 │
│   MB-1R2T       │ TTL  │    FT232RL      │ USB  │    Computer     │
│   LiDAR         │─────▶│    Converter    │─────▶│    (Host)       │
│   Sensor        │ UART │                 │      │                 │
│                 │      │                 │      │                 │
└─────────────────┘      └─────────────────┘      └─────────────────┘
     5V DC                    3.3V/5V                 USB Power
```

---

## LiDAR Sensor: MB-1R2T V1.5.8

### What is LiDAR?

**LiDAR** (Light Detection and Ranging) is a remote sensing technology that uses laser light to measure distances. The MB-1R2T is a **Time-of-Flight (ToF)** based 2D LiDAR that emits infrared laser pulses and measures the time it takes for the light to return after hitting an object.

### Technical Specifications

| Parameter | Value |
|-----------|-------|
| Model | MB-1R2T V1.5.8 |
| Type | 2D 360° Scanning LiDAR |
| Measurement Principle | Time-of-Flight (ToF) |
| Laser Wavelength | 785-905nm (Near-Infrared) |
| Laser Class | Class 1 (Eye-Safe) |
| Range | 0.15m - 12m (typical) |
| Angular Resolution | ~0.5° - 1° |
| Scan Frequency | 5-10 Hz (rotations per second) |
| Angular Range | 360° (full rotation) |
| Operating Voltage | 5V DC |
| Current Consumption | 200-400mA |
| Communication | UART TTL (3.3V/5V compatible) |
| Baud Rate | 153600 bps |

### Internal Components

The MB-1R2T contains several key components:

#### 1. **Laser Diode (TX - Transmitter)**
- Emits short pulses of infrared light (785-905nm wavelength)
- Pulse duration: ~5-10 nanoseconds
- Invisible to human eye but detectable by sensor
- Modulated at high frequency for noise rejection

#### 2. **Photodetector (RX - Receiver)**  
- Avalanche Photodiode (APD) or PIN photodiode
- Extremely sensitive to returning laser light
- Includes optical bandpass filter to reject ambient light
- Converts photons to electrical signal

#### 3. **Rotating Mirror/Prism Assembly**
- Motorized rotating mechanism (typically brushless DC motor)
- Deflects laser beam through 360°
- Encoder tracks exact angular position
- Rotation speed: 5-10 revolutions per second

#### 4. **Motor Driver & Control Circuit**
- PID controller maintains constant rotation speed
- Generates angular position data
- Synchronizes measurements with rotation

#### 5. **Signal Processing Unit (MCU)**
- Typically ARM Cortex-M series microcontroller
- Performs ToF calculations
- Handles UART communication
- Runs firmware for packet formatting

#### 6. **Power Management**
- Voltage regulators (5V to 3.3V, 1.8V)
- Motor driver circuit
- Laser driver circuit with safety cutoffs

### How Time-of-Flight Works

```
          t₀ (laser fires)
             │
             ▼
    ┌────────────────┐
    │   LiDAR TX     │ ═══════▶ Laser Pulse
    └────────────────┘
                              │
                              │ travels at speed of light
                              │ c = 299,792,458 m/s
                              ▼
                        ┌──────────┐
                        │  Object  │
                        └──────────┘
                              │
                              │ reflects back
                              ▼
    ┌────────────────┐
    │   LiDAR RX     │ ◀═══════ Return Pulse
    └────────────────┘
             │
             ▼
          t₁ (pulse detected)

    Distance = (c × Δt) / 2
    
    Where:
    - c = speed of light (299,792,458 m/s)
    - Δt = t₁ - t₀ (round-trip time)
    - Divide by 2 because light travels to object AND back
```

**Example Calculation:**
- Object at 3 meters
- Round-trip distance: 6 meters
- Time: 6m ÷ 299,792,458 m/s = 20.01 nanoseconds
- The sensor measures this incredibly short time interval!

---

## USB-to-Serial Converter: FT232RL

### What is the FT232RL?

The **FT232RL** is a USB to Serial UART interface IC manufactured by **FTDI (Future Technology Devices International)**. It converts USB signals to TTL-level UART signals that the LiDAR sensor understands.

### Technical Specifications

| Parameter | Value |
|-----------|-------|
| Manufacturer | FTDI |
| Interface | USB 2.0 Full Speed |
| USB Speed | 12 Mbps |
| UART Baud Rates | 300 bps to 3 Mbps |
| Logic Levels | 5V or 3.3V (configurable) |
| FIFO Buffers | 128 bytes TX, 256 bytes RX |
| Operating Voltage | 3.3V to 5.25V |
| Current (USB powered) | ~15mA |

### Internal Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                        FT232RL                                │
│                                                               │
│  ┌─────────┐    ┌──────────────┐    ┌─────────────────────┐  │
│  │   USB   │    │   USB        │    │    UART Engine      │  │
│  │ Trans-  │───▶│   Protocol   │───▶│                     │  │
│  │ ceiver  │    │   Engine     │    │  Baud Rate Gen      │  │
│  │         │◀───│              │◀───│  Start/Stop Bits    │  │
│  └─────────┘    └──────────────┘    │  Data Framing       │  │
│       │                              └─────────────────────┘  │
│       │              │                        │               │
│       │         ┌────┴────┐             ┌─────┴─────┐        │
│       │         │  EEPROM │             │   FIFO    │        │
│       │         │  93C46  │             │  Buffers  │        │
│       │         │  Config │             │ TX:128B   │        │
│       │         └─────────┘             │ RX:256B   │        │
│       │                                 └───────────┘        │
│       ▼                                       │               │
│   USB D+/D-                              TXD/RXD              │
│   (to Host)                              (to LiDAR)          │
└──────────────────────────────────────────────────────────────┘
```

### Why FT232RL?

1. **Driver Support**: Built-in drivers on macOS, Windows, Linux
2. **Reliable Timing**: Hardware-based baud rate generation
3. **Buffer Management**: Handles high-speed data without loss
4. **Industry Standard**: Widely used, well-documented

### USB Enumeration Process

When you plug in the FT232RL:

1. **USB Detection** (within 100ms)
   - Host detects device connection via D+/D- voltage change
   - Device draws ≤100mA initially

2. **Enumeration** (within 2 seconds)
   - Host sends GET_DESCRIPTOR requests
   - Device reports VID=0x0403, PID=0x6001 (FTDI standard)
   - Host loads FTDI driver (built-in on modern OS)

3. **Configuration**
   - Driver creates virtual COM port
   - macOS: `/dev/cu.usbserial-XXXXXX`
   - Windows: `COM3`, `COM4`, etc.
   - Linux: `/dev/ttyUSB0`

4. **Ready for Communication**
   - Application opens serial port
   - Sets baud rate (153600 for this LiDAR)
   - Data flows bidirectionally

---

## Physical Connection & Wiring

### Pin Connections

```
MB-1R2T LiDAR          FT232RL Module
─────────────          ──────────────
    VCC  ●────────────● VCC (5V)
    GND  ●────────────● GND
    TX   ●────────────● RX
    RX   ●────────────● TX (optional, not used)

Note: TX/RX are crossed - LiDAR TX connects to FT232RL RX
```

### Voltage Levels

```
TTL UART Signal Levels:

        5V ─┬─────────────────────────────┬─ Logic HIGH (1)
            │                             │
            │    ████████      ████████   │
            │    █      █      █      █   │
            │    █      █      █      █   │
        0V ─┴────█──────█──────█──────█───┴─ Logic LOW (0)
                 │      │      │      │
                 └──────┘      └──────┘
                 
            Start   Data Bits    Stop
            Bit    (LSB first)   Bit
```

### Cable Length Considerations

| Cable Length | Signal Quality | Recommendation |
|--------------|----------------|----------------|
| < 30cm | Excellent | Ideal for prototyping |
| 30cm - 1m | Good | Use quality shielded cable |
| 1m - 3m | Degraded | May need lower baud rate |
| > 3m | Poor | Use RS-485 or signal repeater |

---

## Serial Communication Protocol

### UART Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| Baud Rate | 153600 bps | Bits per second |
| Data Bits | 8 | Payload size per frame |
| Stop Bits | 1 | Frame terminator |
| Parity | None | No error checking bit |
| Flow Control | None | No hardware handshaking |

### Why 153600 Baud?

This unusual baud rate (not a "standard" rate like 115200 or 230400) is chosen because:

1. **Data Throughput Requirements**:
   - LiDAR produces ~400-600 packets/second
   - Each packet: ~120-150 bytes average
   - Required bandwidth: ~80,000 bytes/second minimum
   - 153600 baud ≈ 15,360 bytes/second theoretical max (with overhead)
   - Actually supports ~14,000 bytes/second effective throughput

2. **Crystal Frequency Compatibility**:
   - LiDAR MCU likely uses common crystal (e.g., 24.576 MHz)
   - 153600 = 24,576,000 ÷ 160 (clean division)

### UART Frame Structure

Each byte transmitted follows this pattern:

```
     1 bit    8 bits (LSB first)    1 bit
    ┌─────┬───────────────────────┬──────┐
    │Start│ D0 D1 D2 D3 D4 D5 D6 D7│ Stop │
    │  0  │        DATA           │  1   │
    └─────┴───────────────────────┴──────┘
    
Total: 10 bits per byte (8 data + 1 start + 1 stop)
Effective data rate: 153600 ÷ 10 = 15,360 bytes/second
```

---

## Data Packet Structure

### Complete Packet Format

The MB-1R2T sends data in structured packets, each containing multiple distance measurements:

```
┌────────────────────────────────────────────────────────────────────────┐
│                         COMPLETE PACKET                                 │
├────────┬────────┬────────┬────────┬────────────┬────────────┬─────────┤
│ HEADER │ HEADER │  TYPE  │ COUNT  │START ANGLE │ END ANGLE  │ UNKNOWN │
│  0xAA  │  0x55  │  byte  │  byte  │  2 bytes   │  2 bytes   │ 2 bytes │
├────────┴────────┴────────┴────────┴────────────┴────────────┴─────────┤
│                      MEASUREMENT DATA                                  │
│              (COUNT × 3 bytes per measurement)                         │
│  ┌─────────┬──────────────────────────────────────┐                   │
│  │ Quality │     Distance (16-bit LE)             │ × COUNT           │
│  │ 1 byte  │     2 bytes                          │                   │
│  └─────────┴──────────────────────────────────────┘                   │
└────────────────────────────────────────────────────────────────────────┘

Total packet size: 9 + (COUNT × 3) bytes
Typical packet: 9 + (40 × 3) = 129 bytes
```

### Byte-by-Byte Breakdown

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Header 1 | Always `0xAA` (170 decimal) |
| 1 | 1 | Header 2 | Always `0x55` (85 decimal) |
| 2 | 1 | Type | Packet type identifier |
| 3 | 1 | Count | Number of measurements (N) |
| 4-5 | 2 | Start Angle | Angle × 100, Little-Endian |
| 6-7 | 2 | End Angle | Angle × 100, Little-Endian |
| 8 | 2 | Reserved | Unknown/checksum bytes |
| 9+ | N×3 | Data | Measurement triplets |

### Header Bytes: 0xAA 0x55

The magic bytes `0xAA 0x55` serve as a **sync pattern**:

```
Binary representation:
0xAA = 1010 1010
0x55 = 0101 0101

Combined: 1010 1010 0101 0101

This alternating bit pattern is:
1. Easy to detect in noisy data stream
2. Unlikely to appear randomly in measurement data
3. Self-clocking (helps receiver synchronize)
```

### Angle Encoding

Angles are encoded as 16-bit Little-Endian integers representing **angle × 100** (centidegrees):

```
Example: Angle = 127.45°

Encoded value = 127.45 × 100 = 12745
In hexadecimal: 12745 = 0x31C9

Little-Endian byte order:
Byte 4 (LSB): 0xC9 = 201
Byte 5 (MSB): 0x31 = 49

Raw bytes: C9 31

Decoding:
(201 + 49×256) / 100 = (201 + 12544) / 100 = 127.45°
```

### Measurement Data Structure

Each measurement contains 3 bytes:

```
┌──────────────────────────────────────────────────────────────┐
│                    MEASUREMENT (3 bytes)                     │
├────────────────┬─────────────────────────────────────────────┤
│    QUALITY     │           DISTANCE (Little-Endian)          │
│    Byte 0      │          Byte 1 (LSB)  │  Byte 2 (MSB)      │
├────────────────┼─────────────────────────────────────────────┤
│  Signal        │         Distance in millimeters             │
│  Strength      │         Range: 0 - 65535mm                  │
│  (0-255)       │         (~0 - 65.5 meters)                  │
└────────────────┴─────────────────────────────────────────────┘
```

---

## Byte-Level Protocol Analysis

### Real Packet Example

Here's a real captured packet with full analysis:

```
Raw hex dump:
AA 55 00 28 90 6E A0 7A 00 00 47 D0 03 48 D4 03 
4A DA 03 4B E0 03 4C E6 03 4D EC 03 4E F2 03 50 
F8 03 51 FE 03 52 04 04 53 0A 04 55 10 04 56 16 
04 57 1C 04 58 22 04 59 28 04 5B 2E 04 5C 34 04 
...

Parsed structure:
┌─────────────────────────────────────────────────────────────────┐
│ Bytes 0-1:  AA 55         Header (sync pattern)                 │
│ Byte 2:     00            Packet type                           │
│ Byte 3:     28            Count = 40 measurements               │
│ Bytes 4-5:  90 6E         Start angle: 0x6E90 = 28304 → 283.04° │
│ Bytes 6-7:  A0 7A         End angle: 0x7AA0 = 31392 → 313.92°   │
│ Bytes 8-9:  00 00         Reserved bytes                        │
└─────────────────────────────────────────────────────────────────┘

Measurement data (first 5 of 40):
┌─────┬─────────┬─────────────┬──────────────────────────────────┐
│  #  │ Quality │ Dist (hex)  │ Distance (decoded)               │
├─────┼─────────┼─────────────┼──────────────────────────────────┤
│  1  │   0x47  │   D0 03     │ 0x03D0 = 976mm = 0.976m          │
│  2  │   0x48  │   D4 03     │ 0x03D4 = 980mm = 0.980m          │
│  3  │   0x4A  │   DA 03     │ 0x03DA = 986mm = 0.986m          │
│  4  │   0x4B  │   E0 03     │ 0x03E0 = 992mm = 0.992m          │
│  5  │   0x4C  │   E6 03     │ 0x03E6 = 998mm = 0.998m          │
└─────┴─────────┴─────────────┴──────────────────────────────────┘

Angular step calculation:
Angle range: 313.92° - 283.04° = 30.88°
Measurements: 40
Step: 30.88° / 39 = 0.792° per measurement
```

### Bit-Level Analysis of Distance Value

```
Example: Distance = 986mm = 0x03DA

Binary breakdown:
0x03 = 0000 0011 (MSB - Most Significant Byte)
0xDA = 1101 1010 (LSB - Least Significant Byte)

Little-Endian storage (LSB first):
Memory: [DA] [03]
         │     │
         │     └── Byte at offset+2
         └──────── Byte at offset+1

16-bit value reconstruction:
LSB + (MSB × 256)
= 0xDA + (0x03 × 0x100)
= 218 + 768
= 986 mm
```

### Invalid Reading Detection

Certain values indicate invalid measurements:

```
INVALID READINGS:
─────────────────

1. Distance ≥ 60000mm (0xEA60)
   - Indicates: No return signal (object too far or no reflection)
   - Example: 65240mm (0xFED8) - typical "no object" value
   
2. Distance = 0
   - Indicates: Object too close (below minimum range)
   - Minimum range typically ~150mm

3. Quality ≤ 5
   - Indicates: Weak return signal, unreliable measurement
   - Causes: Transparent/dark objects, extreme angles
   
Common invalid patterns:
┌─────────────┬────────────┬──────────────────────────┐
│ Quality     │ Distance   │ Interpretation           │
├─────────────┼────────────┼──────────────────────────┤
│ 0x01 (1)    │ 0xFA00     │ No object detected       │
│ 0x00 (0)    │ 0xFFFF     │ Sensor error             │
│ 0x05 (5)    │ 0xEA60     │ Very weak reflection     │
│ 0x02 (2)    │ 0xF0A0     │ Out of range             │
└─────────────┴────────────┴──────────────────────────┘
```

---

## Distance Measurement Principles

### Time-of-Flight Calculation Details

```
Physical Constants:
─────────────────────
Speed of light (c): 299,792,458 m/s
                    ≈ 0.2998 m/ns (meters per nanosecond)
                    ≈ 0.2998 mm/ps (millimeters per picosecond)

Time Resolution Required:
─────────────────────────
For 1mm accuracy:
Round-trip distance: 2mm
Time = 2mm ÷ 299,792,458,000 mm/s
     = 6.67 picoseconds

The sensor must measure time intervals of ~7 picoseconds!
This is achieved using specialized Time-to-Digital Converters (TDC).
```

### Measurement Process

```
    Time ──────────────────────────────────────────────────────▶

    ┌───┐
    │TX │ Laser pulse fired at t₀
    └─┬─┘
      │
      └──── Photons travel outward ────▶ ●●●●●●●●●●●●●●●●●●●●●●
                                                              │
                                                    ┌─────────┴───┐
                                                    │   Target    │
                                                    │   Object    │
                                                    └─────────┬───┘
                                                              │
           ◀──── Photons return ──── ●●●●●●●●●●●●●●●●●●●●●●───┘
      │
    ┌─┴─┐
    │RX │ Return pulse detected at t₁
    └───┘

    Measurement:
    Δt = t₁ - t₀
    Distance = (c × Δt) / 2
```

### Sources of Error

| Error Source | Magnitude | Mitigation |
|--------------|-----------|------------|
| Clock jitter | ±1-5mm | High-quality crystal, averaging |
| Temperature drift | ±2mm/°C | Internal compensation |
| Target reflectivity | ±1-10mm | Quality-based filtering |
| Multi-path reflection | Variable | Signal processing |
| Atmospheric conditions | <0.1mm/m | Generally negligible |

---

## Signal Quality & Filtering

### Quality Value Interpretation

The quality byte (0-255) represents the signal-to-noise ratio of the return:

```
Quality Ranges:
───────────────

  0-5:    ████░░░░░░░░░░░░░░░░  Very Poor - Reject
  6-30:   ████████░░░░░░░░░░░░  Poor - Use with caution
 31-80:   ████████████░░░░░░░░  Fair - Acceptable
 81-150:  ████████████████░░░░  Good - Reliable
151-200:  ████████████████████  Excellent - High confidence
201-255:  ████████████████████  Maximum - Retroreflector or close object

Factors affecting quality:
┌──────────────────────┬───────────────────────────────────┐
│ Factor               │ Effect on Quality                 │
├──────────────────────┼───────────────────────────────────┤
│ Distance             │ Decreases with distance (1/r²)    │
│ Surface color        │ Dark surfaces = lower quality     │
│ Surface angle        │ Glancing angles = lower quality   │
│ Surface material     │ Matte > Glossy > Transparent      │
│ Ambient light        │ Sunlight reduces quality          │
│ Retroreflector       │ Very high quality (>200)          │
└──────────────────────┴───────────────────────────────────┘
```

### Recommended Filtering Strategy

```python
def is_valid_measurement(quality, distance):
    """
    Filter invalid LiDAR measurements
    
    Returns True if measurement should be used
    """
    # Reject very low quality (noise)
    if quality <= 5:
        return False
    
    # Reject "no return" distances
    if distance >= 60000:  # 60+ meters
        return False
    
    # Reject zero distance (too close)
    if distance == 0:
        return False
    
    # Optional: Reject suspiciously high quality at long range
    # (might be cross-talk or interference)
    if distance > 10000 and quality > 200:
        return False  # 10m with max quality is suspicious
    
    return True
```

---

## Timing & Data Rates

### Data Flow Analysis

```
LiDAR Rotation Speed: ~7 Hz (7 rotations per second)
Measurements per rotation: ~500-720 points (0.5°-1° resolution)
Measurements per second: ~3500-5000 points

Packet Statistics:
─────────────────
Packets per rotation: ~12-18
Measurements per packet: 30-40 (typical)
Packets per second: ~100-150
Bytes per packet: ~100-130
Total data rate: ~12,000-18,000 bytes/second

Serial Port Utilization:
────────────────────────
Baud rate: 153600 bps
Max throughput: ~15,360 bytes/second
Typical usage: ~80-90% of capacity
```

### Timing Diagram

```
Time (ms)
0    10   20   30   40   50   60   70   80   90   100  110  120  130  140
│    │    │    │    │    │    │    │    │    │    │    │    │    │    │
├────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┤
│                        One LiDAR Rotation (~143ms @ 7Hz)             │
├──────────────────────────────────────────────────────────────────────┤

Packets during rotation:
│▓▓│  │▓▓│  │▓▓│  │▓▓│  │▓▓│  │▓▓│  │▓▓│  │▓▓│  │▓▓│  │▓▓│  │▓▓│  │▓▓│
 P1    P2    P3    P4    P5    P6    P7    P8    P9   P10   P11   P12

Each packet (▓▓):
- Duration: ~8-10ms transmission time
- Contains: ~35-40 measurements
- Covers: ~25-30° of rotation
```

---

## Software Implementation

### Reading Data (Python)

```python
import serial

# Open serial port
ser = serial.Serial(
    port='/dev/cu.usbserial-A5069RR4',  # macOS
    # port='COM3',                       # Windows
    baudrate=153600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0  # Non-blocking
)

def parse_packet(buffer):
    """
    Parse a complete LiDAR packet from buffer
    Returns list of (angle, distance, quality) tuples
    """
    points = []
    
    # Verify header
    if buffer[0] != 0xAA or buffer[1] != 0x55:
        return None
    
    # Extract packet info
    packet_type = buffer[2]
    num_measurements = buffer[3]
    
    # Decode angles (16-bit Little-Endian, value = degrees × 100)
    start_angle = (buffer[4] | (buffer[5] << 8)) / 100.0
    end_angle = (buffer[6] | (buffer[7] << 8)) / 100.0
    
    # Handle wraparound (e.g., 350° to 10°)
    if end_angle < start_angle:
        end_angle += 360.0
    
    # Calculate angle step
    if num_measurements > 1:
        angle_step = (end_angle - start_angle) / (num_measurements - 1)
    else:
        angle_step = 0
    
    # Parse each measurement
    for i in range(num_measurements):
        offset = 9 + i * 3
        
        quality = buffer[offset]
        distance = buffer[offset + 1] | (buffer[offset + 2] << 8)
        
        # Filter invalid readings
        if distance < 60000 and quality > 5:
            angle = (start_angle + i * angle_step) % 360
            points.append((angle, distance, quality))
    
    return points
```

### Packet Synchronization

```python
def find_packet_start(buffer):
    """
    Find the start of a valid packet in buffer
    Returns index of packet start, or -1 if not found
    """
    for i in range(len(buffer) - 1):
        if buffer[i] == 0xAA and buffer[i + 1] == 0x55:
            # Verify we have enough bytes to read header
            if i + 9 <= len(buffer):
                num_measurements = buffer[i + 3]
                packet_length = 9 + num_measurements * 3
                
                # Verify packet length is reasonable
                if num_measurements <= 100:
                    return i
    
    return -1
```

---

## Troubleshooting

### Common Issues

| Issue | Symptoms | Solution |
|-------|----------|----------|
| No data | Port opens but no packets | Check wiring, verify 5V power |
| Corrupted data | Malformed packets, CRC errors | Check baud rate (must be 153600) |
| Port locked | "Resource busy" error | Kill other processes, replug USB |
| Intermittent | Works then stops | Buffer overflow, process faster |
| Missing points | Gaps in scan | Normal - some surfaces don't reflect |

### Diagnostic Commands

**macOS:**
```bash
# List USB devices
ls /dev/cu.usb*

# Check USB info
system_profiler SPUSBDataType | grep -A10 "Serial"

# Monitor serial data (raw)
cat /dev/cu.usbserial-* | xxd | head -100
```

**Windows:**
```powershell
# List COM ports
Get-WMIObject Win32_SerialPort

# Check device manager
devmgmt.msc
```

**Linux:**
```bash
# List USB serial devices
ls /dev/ttyUSB*

# Check kernel messages
dmesg | grep tty

# Monitor serial data
cat /dev/ttyUSB0 | xxd | head -100
```

---

## Appendix A: Hexadecimal Reference

```
Common values you'll see in packets:

Header:     AA 55     (Start of packet marker)
Type:       00        (Standard measurement packet)
Count:      28        (40 measurements in hex)

Angle examples (Little-Endian):
00 00 → 0.00°
E8 03 → 10.00°     (0x03E8 = 1000 centidegrees)
D0 07 → 20.00°     (0x07D0 = 2000)
10 27 → 100.00°    (0x2710 = 10000)
20 4E → 200.00°    (0x4E20 = 20000)
30 75 → 300.00°    (0x7530 = 30000)

Distance examples (Little-Endian):
E8 03 → 1000mm = 1.0m
D0 07 → 2000mm = 2.0m
10 27 → 10000mm = 10.0m

Invalid markers:
60 EA → 60000mm (out of range marker)
FF FF → 65535mm (no reflection)
```

---

## Appendix B: References

1. FTDI FT232R Datasheet
2. ToF LiDAR Operating Principles
3. UART Protocol Specification
4. USB 2.0 Specification
5. MB-1R2T Manufacturer Documentation (Chinese)

---

*Documentation created for the MB-1R2T LiDAR + FT232RL visualization project*
*Last updated: February 2026*
