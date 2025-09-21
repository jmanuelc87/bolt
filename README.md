# Bolt Firmware

The Protocol:

```
[SOF=0xAA][TYPE:1][LEN:1][PAYLOAD:LEN][CRC16-CCITT:2][EOF=0x55]
```

Ping Frame:

```
AA 01 00 2E 3E 55
```

Set Motor Pulse Frame:

motor_id == 1 values (1,2,3,4)
pulse == 2050 ranges [0, 2000]

```
AA 02 05 01 00 00 08 02 D1 96 55
```
