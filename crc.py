def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """
    Compute CRC16-CCITT (polynomial 0x1021, initial value 0xFFFF).
    """
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


data = bytes([0x04, 0x02, 0x00, 0x00])
print(hex(crc16_ccitt(data)))
