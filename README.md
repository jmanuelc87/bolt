# Bolt Firmware

The Protocol:

```
[SOF=0xAA][TYPE:1][LEN:1][PAYLOAD:LEN][CRC16-CCITT:2][EOF=0x55]
```

#### Ping Frame:

```
AA 01 00 2E 3E 55
```

#### Set Motor Pulse Frame:

motor_id == 1 values (1,2,3,4)
pulse == 1000 ranges [0, 2000]

```
AA 02 03 01 03 E8 D0 16 55
```

#### Motor Stop Frame

motor_id == 1 values (1,2,3,4)
brake == 0 or 1

```
AA 03 02 01 00 42 4D 55
```


#### PWM Servo Move Frame

servo_id == 0 values (0, 1, 2, 3)
angle == 60 ranges [0, 180]

```
AA 04 02 00 3C D7 8E 55
```
