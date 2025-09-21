# Bolt Firmware

The Protocol:

```
[SOF=0xAA][TYPE:1][LEN:1][PAYLOAD:LEN][CRC16-CCITT:2][EOF=0x55]
```

Ping Frame:

```
AA 01 00 2E 3E 55
```