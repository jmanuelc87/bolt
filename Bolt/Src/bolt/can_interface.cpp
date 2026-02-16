#include "interface/can_interface.hpp"

#include <cstring>

static struct
{
    uint8_t buf[RX_BUF_MAX];
    uint16_t expect_len;
    uint16_t have;
    uint8_t next_sn; // next sequence number (0..15)
    uint8_t active;  // 0=idle, 1=reassembling
} rx;

static struct
{
    volatile uint8_t cts;   // 1 when we may continue sending CFs
    volatile uint8_t stmin; // inter-frame separation (ms), 0=asap
} tx;

void bolt::can::CanBusAsyncPort::isotpSendFcCts(void)
{
    // FC frame: [PCI=0x30][BS=0][STmin=0] then padding unused
    uint8_t fc[8] = {(uint8_t)((PCI_FC << 4) | FC_CTS), 0x00, 0x00, 0, 0, 0, 0, 0};
    sendMessage(CAN_ID_FC, fc, 3);
}

void bolt::can::CanBusAsyncPort::sendMessage(uint32_t stdid, const uint8_t *d, uint8_t dlc)
{
    CAN_TxHeaderTypeDef th;
    th.StdId = stdid;
    th.IDE = CAN_ID_STD;
    th.RTR = CAN_RTR_DATA;
    th.DLC = dlc;
    th.TransmitGlobalTime = DISABLE;
    uint32_t mb;
    HAL_CAN_AddTxMessage(hcan_, &th, (uint8_t *)d, &mb);
}

void bolt::can::CanBusAsyncPort::isotpSend(const uint8_t *data, uint16_t len)
{
    if (len == 0)
        return;

    if (len <= 7)
    {
        // Single frame
        uint8_t sf[8] = {(uint8_t)((PCI_SF << 4) | (len & 0x0F))};
        memcpy(&sf[1], data, len);
        sendMessage(CAN_TX_ID_DATA, sf, (uint8_t)(len + 1));
        return;
    }

    // Multi-frame: First Frame
    uint8_t ff[8];
    ff[0] = (uint8_t)((PCI_FF << 4) | ((len >> 8) & 0x0F));
    ff[1] = (uint8_t)(len & 0xFF);
    uint16_t off = 0;
    const uint8_t first_payload = 6;
    memcpy(&ff[2], &data[off], first_payload);
    off += first_payload;

    // Wait for Flow Control (CTS)
    // Clear before sending FF to avoid TOCTOU race if FC arrives immediately
    tx.cts = 0;
    tx.stmin = 0;
    sendMessage(CAN_TX_ID_DATA, ff, 8);

    uint32_t start = HAL_GetTick();
    while (!tx.cts && (HAL_GetTick() - start) < TX_TIMEOUT_MS)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!tx.cts)
        return; // FC(CTS) not received — abort send

    // Send Consecutive Frames
    uint8_t sn = 1;
    while (off < len)
    {
        uint8_t cf[8];
        cf[0] = (uint8_t)((PCI_CF << 4) | (sn & 0x0F));
        uint8_t n = (uint8_t)((len - off) > 7 ? 7 : (len - off));
        memcpy(&cf[1], &data[off], n);
        off += n;
        sendMessage(CAN_TX_ID_DATA, cf, (uint8_t)(n + 1));

        // next sequence number with wrap 0..15
        sn = (uint8_t)((sn + 1) & 0x0F);

        // Respect STmin (0..127 ms). Ignore microsecond encodings (0xF1..0xF9).
        if (tx.stmin > 0 && tx.stmin <= 0x7F)
        {
            HAL_Delay(tx.stmin);
        }
    }
}

// Call this from your CAN RX callback when a frame with StdId==CAN_RX_ID_DATA arrives
void bolt::can::CanBusAsyncPort::isotpRxOnCan(uint8_t *data, uint8_t dlc)
{
    if (dlc == 0)
        return;
    uint8_t pci = data[0] >> 4;

    switch (pci)
    {
    case PCI_SF:
    {
        uint8_t sfl = data[0] & 0x0F; // len (0..7)
        if (sfl > 7 || sfl > dlc - 1)
            return;

        if (this->onMessageRecieved)
        {
            this->onMessageRecieved(&data[1], sfl);
        }
        break;
    }

    case PCI_FF:
    {
        if (dlc < 8)
            return;
        uint16_t len = ((data[0] & 0x0F) << 8) | data[1]; // 12-bit length
        if (len == 0 || len > RX_BUF_MAX)
            return;

        rx.expect_len = len;
        rx.have = 0;
        rx.next_sn = 1; // CF sequence starts at 1
        rx.active = 1;

        uint8_t first_payload = 6; // bytes in FF payload (DLC 8: bytes 2..7)
        memcpy(&rx.buf[0], &data[2], first_payload);
        rx.have += first_payload;

        isotpSendFcCts(); // tell sender to go ahead
        break;
    }

    case PCI_CF:
    {
        if (!rx.active || dlc < 2)
            return;
        uint8_t sn = data[0] & 0x0F;
        if (sn != rx.next_sn)
        {
            // out of order — drop
            rx.active = 0;
            return;
        }
        uint8_t n = (uint8_t)((rx.expect_len - rx.have) > 7 ? 7 : (rx.expect_len - rx.have));
        if (n > (dlc - 1))
            n = dlc - 1; // safety
        memcpy(&rx.buf[rx.have], &data[1], n);
        rx.have += n;
        rx.next_sn = (uint8_t)((rx.next_sn + 1) & 0x0F);

        if (rx.have >= rx.expect_len)
        {
            if (this->onMessageRecieved)
            {
                this->onMessageRecieved(rx.buf, rx.expect_len);
            }
            rx.active = 0;
        }
        break;
    }

    default:
        break;
    }
}

void bolt::can::CanBusAsyncPort::isotpRxOnFc(uint8_t *data, uint8_t dlc)
{
    if (dlc < 3)
        return;
    if ((data[0] >> 4) != PCI_FC)
        return;

    uint8_t fs = data[0] & 0x0F;

    switch (fs)
    {
    case FC_CTS:
        tx.cts = 1;
        tx.stmin = data[2];
        break;
    case FC_WAIT:
        // Could extend timeout here; currently ignored.
        break;
    case FC_OVFL:
    default:
        // Abort current TX
        tx.cts = 0;
        break;
    }
}
