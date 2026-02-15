# Bolt Firmware Protocol

All communication between the host and the board uses a compact binary frame protocol. Every frame follows the same envelope structure, making parsing straightforward and consistent across all command types.

## Frame Structure

```
[SOF][TYPE][LEN][PAYLOAD][CRC16][EOF]
```

| Field   | Size (bytes) | Description                                      |
|---------|:------------:|--------------------------------------------------|
| SOF     | 1            | Start-of-frame marker, always `0xAA`             |
| TYPE    | 1            | Frame type identifier (see table below)          |
| LEN     | 1            | Length of the payload in bytes                    |
| PAYLOAD | LEN          | Type-specific data (may be empty)                |
| CRC16   | 2            | CRC16-CCITT computed over TYPE + LEN + PAYLOAD   |
| EOF     | 1            | End-of-frame marker, always `0x55`               |

All multi-byte integers are encoded **big-endian**.

## Frame Types

| Type | Name                | Direction    | Description                          |
|:----:|---------------------|:------------:|--------------------------------------|
| 0x01 | Ping                | Host -> Board | Health check; board echoes a reply  |
| 0x02 | Set Motor Pulse     | Host -> Board | Set speed/direction for a DC motor  |
| 0x03 | Motor Stop          | Host -> Board | Stop a DC motor (coast or brake)    |
| 0x04 | PWM Servo Move      | Host -> Board | Move a PWM servo to an angle        |
| 0x05 | Serial Servo Move   | Host -> Board | Move a serial servo to a position   |
| 0x06 | Serial Servo Angle  | Host -> Board | Request current angle of a servo    |
| 0x07 | Get Encoder Values  | Host -> Board | Request encoder readings            |
| 0x08 | Get IMU Values      | Host -> Board | Request IMU sensor readings         |
| 0x09 | PID Motor Set RPM   | Host -> Board | Set target RPM for PID motor control|
| 0x0A | PID Motor Stop      | Host -> Board | Stop PID motor control              |
| 0x0B | PID Set Gains       | Host -> Board | Set PID gains (Kp, Ki, Kd)          |

---

## Frame Details

### 0x01 -- Ping

Empty payload. Used to verify the board is alive and responding.

```
AA 01 00 2E 3E 55
```

---

### 0x02 -- Set Motor Pulse

Sets the PWM pulse for a DC motor, controlling its speed and direction.

| Field    | Size | Range      | Description             |
|----------|:----:|------------|-------------------------|
| motor_id | 1    | 1 -- 4     | Motor channel           |
| pulse    | 2    | 0 -- 2000  | PWM pulse value         |

**Example** -- Motor 1 at pulse 1000:

```
AA 02 03 01 03 E8 D0 16 55
```

---

### 0x03 -- Motor Stop

Stops a DC motor. The brake flag selects between coasting (0) and active braking (1).

| Field    | Size | Range  | Description                    |
|----------|:----:|--------|--------------------------------|
| motor_id | 1    | 1 -- 4 | Motor channel                  |
| brake    | 1    | 0 or 1 | 0 = coast, 1 = active brake   |

**Example** -- Stop motor 1, coast:

```
AA 03 02 01 00 42 4D 55
```

---

### 0x04 -- PWM Servo Move

Moves a PWM servo to an absolute angle.

| Field    | Size | Range     | Description             |
|----------|:----:|-----------|-------------------------|
| servo_id | 1    | 0 -- 3    | Servo channel           |
| angle    | 1    | 0 -- 180  | Target angle in degrees |

**Example** -- Servo 0 to 60 degrees:

```
AA 04 02 00 3C D7 8E 55
```

---

### 0x05 -- Serial Servo Move

Moves a serial bus servo to a target position over a given duration.

| Field    | Size | Range        | Description                    |
|----------|:----:|--------------|--------------------------------|
| servo_id | 1    | 1 -- 254     | Servo ID on the serial bus     |
| pulse    | 2    | 96 -- 3999   | Target position (pulse width)  |
| time     | 2    | 0 -- 2000    | Travel duration in ms          |

**Example** -- Servo 1 to pulse 2000 in 500 ms:

```
AA 05 05 01 07 D0 01 F4 C9 2A 55
```

---

### 0x06 -- Serial Servo Get Angle

Requests the current angle from a serial bus servo. The board responds with the reading.

| Field    | Size | Range    | Description                    |
|----------|:----:|----------|--------------------------------|
| servo_id | 1    | 1 -- 254 | Servo ID on the serial bus     |

**Example** -- Query servo 1:

```
AA 06 01 01 5D 2C 55
```

---

### 0x07 -- Get Encoder Values

Requests the current encoder tick count for a motor. The board responds with the accumulated value.

| Field    | Size | Range  | Description        |
|----------|:----:|--------|--------------------|
| motor_id | 1    | 0 -- 3 | Encoder channel    |

**Example** -- Query all encoders (no payload):

```
AA 07 00 84 98 55
```

---

### 0x08 -- Get IMU Values

Requests accelerometer, gyroscope, magnetometer, and temperature readings from the ICM20948 IMU. Empty payload.

The board responds with a 20-byte payload containing 10 signed 16-bit integers (big-endian):

| Field   | Size | Description                |
|---------|:----:|----------------------------|
| accel_x | 2    | Accelerometer X axis       |
| accel_y | 2    | Accelerometer Y axis       |
| accel_z | 2    | Accelerometer Z axis       |
| gyro_x  | 2    | Gyroscope X axis           |
| gyro_y  | 2    | Gyroscope Y axis           |
| gyro_z  | 2    | Gyroscope Z axis           |
| mag_x   | 2    | Magnetometer X axis        |
| mag_y   | 2    | Magnetometer Y axis        |
| mag_z   | 2    | Magnetometer Z axis        |
| temp    | 2    | Temperature                |

Response frame type: `0x04` (IMU)

**Example** -- Request IMU values:

```
AA 08 00 A6 D8 55
```

---

### 0x09 -- PID Motor Set RPM

Sets a target RPM for closed-loop PID motor control. The PID controller reads encoder RPM as feedback and drives the motor PWM output to reach the target.

| Field    | Size | Range       | Description                              |
|----------|:----:|-------------|------------------------------------------|
| motor_id | 1    | 1 -- 4      | Motor channel                            |
| rpm      | 4    | IEEE-754    | Target RPM (float, little-endian bytes)  |

**Example** -- Motor 1 at 120.0 RPM (float `0x42F00000`):

```
AA 09 05 01 00 00 F0 42 48 15 55
```

---

### 0x0A -- PID Motor Stop

Stops PID motor control and optionally applies braking. Resets the PID integrator state.

| Field    | Size | Range  | Description                    |
|----------|:----:|--------|--------------------------------|
| motor_id | 1    | 1 -- 4 | Motor channel                  |
| brake    | 1    | 0 or 1 | 0 = coast, 1 = active brake   |

**Example** -- Stop PID motor 1, coast:

```
AA 0A 02 01 00 B1 3A 55
```

---

### 0x0B -- PID Set Gains

Sets the PID controller gains (Kp, Ki, Kd) for a specific motor channel. All gains are IEEE-754 floats in little-endian byte order.

| Field    | Size | Range    | Description                              |
|----------|:----:|----------|------------------------------------------|
| motor_id | 1    | 1 -- 4   | Motor channel                            |
| kp       | 4    | IEEE-754 | Proportional gain (float, little-endian) |
| ki       | 4    | IEEE-754 | Integral gain (float, little-endian)     |
| kd       | 4    | IEEE-754 | Derivative gain (float, little-endian)   |

**Example** -- Motor 1, Kp=1.0, Ki=0.1, Kd=0.01:

```
AA 0B 0D 01 00 00 80 3F CD CC CC 3D 0A D7 23 3C E7 C4 55
```
