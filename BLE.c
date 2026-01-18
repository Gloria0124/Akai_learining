//******************************************************************************
//  * @file           : BLE.c
//  ******************************************************************************

#include "BLE.h"
static uint8_t ble_rx_buf[BLE_RX_BUF_LEN] = {0};
static uint16_t ble_rx_len = 0;
static uint8_t ble_temp_byte;

/**
 * @brief  启动蓝牙串口接收
 */
void BLE_StartRx(void)
{
    
    HAL_UART_Receive_IT(&huart3, &ble_temp_byte, 1);
}


void BLE_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart3, &Byte, 1, 10);
}

void BLE_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit(&huart3, Array, Length, 100);
}

void BLE_SendString(char *String)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)String, strlen(String), 100);
}


void BLE_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    BLE_SendString(String);
}


static uint8_t BLE_CalcChecksum(const uint8_t *pData, uint16_t len)
{
    uint8_t sum = 0;
    if (pData == NULL || len == 0) return 0;
    for (uint16_t i = 0; i < len; i++) sum += pData[i];
    return sum;
}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == USART3) 
//    {
//       
//        if(ble_rx_len < BLE_RX_BUF_LEN)
//        {
//            ble_rx_buf[ble_rx_len++] = ble_temp_byte;
//        }
//        
//     
//        HAL_UART_Receive_IT(&huart3, &ble_temp_byte, 1);
//    }
//}


int8_t BLE_SendFloatArrayPacket(float *float_array, uint8_t array_len)
{
    if (float_array == NULL || array_len == 0 || array_len > MAX_FLOAT_CNT) return -1;
    
    uint16_t float_total_bytes = array_len * FLOAT_BYTE_LEN;
    uint16_t total_pkt_len = 1 + float_total_bytes + 1 + 1;
    uint8_t packet[64] = {0}; 

    packet[0] = PACKET_HEAD;
    
    uint8_t *float_byte_ptr = &packet[1];
    for (uint8_t i = 0; i < array_len; i++) {
        memcpy(float_byte_ptr + (i * FLOAT_BYTE_LEN), &float_array[i], FLOAT_BYTE_LEN);
    }
    
    packet[1 + float_total_bytes] = BLE_CalcChecksum(float_byte_ptr, float_total_bytes);
    packet[total_pkt_len - 1] = PACKET_TAIL;

    BLE_SendArray(packet, total_pkt_len);
    return 0;
}

int8_t BLE_RecvFloatArrayPacket(float *float_array)
{
    if (float_array == NULL) return -1;
    

    uint16_t pkt_start = BLE_RX_BUF_LEN;
    for (uint16_t i = 0; i < ble_rx_len; i++)
    {
        if (ble_rx_buf[i] == PACKET_HEAD)
        {
            pkt_start = i;
            break;
        }
    }
    if (pkt_start == BLE_RX_BUF_LEN)
    {
        ble_rx_len = 0;
        memset(ble_rx_buf, 0, BLE_RX_BUF_LEN);
        return -2;
    }
    

    uint16_t pkt_end = BLE_RX_BUF_LEN;
    for (uint16_t j = pkt_start + 1; j < ble_rx_len; j++)
    {
        if (ble_rx_buf[j] == PACKET_TAIL)
        {
            pkt_end = j;
            break;
        }
    }
    if (pkt_end == BLE_RX_BUF_LEN) return -2; 
    

    uint16_t total_pkt_len = pkt_end - pkt_start + 1;
    if (total_pkt_len < 7 || (total_pkt_len - 3) % FLOAT_BYTE_LEN != 0)
    {

        memmove(ble_rx_buf, ble_rx_buf + pkt_start + 1, ble_rx_len - pkt_start - 1);
        ble_rx_len -= pkt_start + 1;
        return -4;
    }
    
    uint8_t float_cnt = (total_pkt_len - 3) / FLOAT_BYTE_LEN;
    if (float_cnt == 0 || float_cnt > MAX_FLOAT_CNT)
    {
        memmove(ble_rx_buf, ble_rx_buf + pkt_end + 1, ble_rx_len - pkt_end - 1);
        ble_rx_len -= pkt_end + 1;
        return -4;
    }


    uint8_t *float_byte_ptr = &ble_rx_buf[pkt_start + 1];
    uint8_t recv_checksum = ble_rx_buf[pkt_start + 1 + float_cnt*FLOAT_BYTE_LEN];
    uint8_t calc_checksum = BLE_CalcChecksum(float_byte_ptr, float_cnt*FLOAT_BYTE_LEN);
    
    if (recv_checksum != calc_checksum)
    {

        memmove(ble_rx_buf, ble_rx_buf + pkt_end + 1, ble_rx_len - pkt_end - 1);
        ble_rx_len -= pkt_end + 1;
        return -3;
    }

    for (uint8_t i = 0; i < float_cnt; i++)
    {
        uint8_t *float_bytes = &float_byte_ptr[i*FLOAT_BYTE_LEN];
        memcpy(&float_array[i], float_bytes, FLOAT_BYTE_LEN);
    }


    memmove(ble_rx_buf, ble_rx_buf + pkt_end + 1, ble_rx_len - pkt_end - 1);
    ble_rx_len -= pkt_end + 1;
    
    return float_cnt;
}


int8_t BLE_SendInt8ArrayPacket(int8_t *int8_array, uint8_t array_len)
{
    if (int8_array == NULL || array_len == 0 || array_len > MAX_INT8_CNT) return -1;

    uint16_t int8_total_bytes = array_len * INT8_BYTE_LEN;
    uint16_t total_pkt_len = 1 + int8_total_bytes + 1 + 1; 
    
    uint8_t packet[64] = {0}; 

    packet[0] = PACKET_HEAD;


    uint8_t *int8_byte_ptr = &packet[1];
    
    for (uint8_t i = 0; i < array_len; i++) 
    {

        int8_byte_ptr[i] = (uint8_t)int8_array[i];
    }


    packet[1 + int8_total_bytes] = BLE_CalcChecksum(int8_byte_ptr, int8_total_bytes);
    

    packet[total_pkt_len - 1] = PACKET_TAIL;


    BLE_SendArray(packet, total_pkt_len);
    
    return 0;
}


int8_t BLE_RecvInt8ArrayPacket(int8_t *int8_array)
{
    if (int8_array == NULL) return -1;


    uint16_t pkt_start = BLE_RX_BUF_LEN;
    for (uint16_t i = 0; i < ble_rx_len; i++)
    {
        if (ble_rx_buf[i] == PACKET_HEAD)
        {
            pkt_start = i;
            break;
        }
    }

    if (pkt_start == BLE_RX_BUF_LEN)
    {
        ble_rx_len = 0;
        memset(ble_rx_buf, 0, BLE_RX_BUF_LEN);
        return -2; 
    }


    uint16_t pkt_end = BLE_RX_BUF_LEN;
    for (uint16_t j = pkt_start + 1; j < ble_rx_len; j++)
    {
        if (ble_rx_buf[j] == PACKET_TAIL)
        {
            pkt_end = j;
            break;
        }
    }
    if (pkt_end == BLE_RX_BUF_LEN) return -2; 

    uint16_t total_pkt_len = pkt_end - pkt_start + 1;

    if (total_pkt_len < 4 || (total_pkt_len - 3) % INT8_BYTE_LEN != 0)
    {

        memmove(ble_rx_buf, ble_rx_buf + pkt_start + 1, ble_rx_len - pkt_start - 1);
        ble_rx_len -= pkt_start + 1;
        return -4;
    }


    uint8_t int8_cnt = (total_pkt_len - 3) / INT8_BYTE_LEN;
    if (int8_cnt == 0 || int8_cnt > MAX_INT8_CNT) 
    {
        memmove(ble_rx_buf, ble_rx_buf + pkt_end + 1, ble_rx_len - pkt_end - 1);
        ble_rx_len -= pkt_end + 1;
        return -4; 
    }

    uint8_t *int8_byte_ptr = &ble_rx_buf[pkt_start + 1];
    uint8_t recv_checksum = ble_rx_buf[pkt_start + 1 + int8_cnt * INT8_BYTE_LEN]; 
    uint8_t calc_checksum = BLE_CalcChecksum(int8_byte_ptr, int8_cnt * INT8_BYTE_LEN);
    
    if (recv_checksum != calc_checksum)
    {
        memmove(ble_rx_buf, ble_rx_buf + pkt_end + 1, ble_rx_len - pkt_end - 1);
        ble_rx_len -= pkt_end + 1;
        return -3;
    }

    for (uint8_t i = 0; i < int8_cnt; i++)
    {

        int8_array[i] = (int8_t)int8_byte_ptr[i];
    }


    memmove(ble_rx_buf, ble_rx_buf + pkt_end + 1, ble_rx_len - pkt_end - 1);
    ble_rx_len -= pkt_end + 1;
    
    return int8_cnt; 
}
