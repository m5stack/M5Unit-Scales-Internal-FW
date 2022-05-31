/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include <string.h>
#include <stdlib.h>
#include "stdint.h"
#include "i2c_ex.h"
#include "sk6812.h"
#include "main.h"
#include "flash.h"

extern volatile uint32_t rgb_color;
extern volatile uint8_t flag_rgb_color;
extern uint8_t g_scale_set[];
extern volatile uint16_t g_scale_set_0;
extern volatile uint16_t g_scale_set_load;
extern volatile uint16_t flag_scale_set_0;
extern volatile uint16_t flag_scale_set_load;
extern volatile uint32_t g_adc_0;
extern volatile uint32_t g_adc_load;
uint8_t data_buf[4] = {0};

typedef struct _I2CReg_t {
    uint8_t address;
    uint8_t *user_buff;
    uint8_t *i2c_buff;
    uint8_t bit;
    uint8_t len;
    struct _I2CReg_t *next;
    /* data */
} I2CReg_t;

I2CReg_t *reg_list = NULL;

__IO uint8_t rx_buffer[I2C_RECEIVE_BUFFER_LEN];
__IO uint8_t tx_buffer[I2C_RECEIVE_BUFFER_LEN];
__IO uint16_t tx_len  = 0;
__IO uint8_t tx_state = 0;

static void I2CRead(uint8_t reg);
static void I2CWrite(uint8_t reg, uint8_t *data, uint8_t len);

__weak void i2c2_receive_callback(uint8_t *rx_data, uint16_t len) {
    /* Prevent unused argument(s) compilation warning */
    if (len == 1) {
        if ((rx_data[0] >= 0x50) && (rx_data[0] <= 0x52)) {
            data_buf[0] = rgb_color & 0xff;
            data_buf[1] = (rgb_color >> 8) & 0xff;
            data_buf[2] = (rgb_color >> 16) & 0xff;
            data_buf[3] = 0;
            i2c2_set_send_data(data_buf, 3);
        } else if ((rx_data[0] >= 0x1C) && (rx_data[0] <= 0x1D)) {
            return;
        } else if ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x31)) {
            i2c2_set_send_data(g_scale_set, 2);
        } else if ((rx_data[0] >= 0x32) && (rx_data[0] <= 0x33)) {
            i2c2_set_send_data((uint8_t *)&g_scale_set[2], 2);
        } else if (rx_data[0] == 0xFF) {
            i2c2_set_send_data(flash_arg, 1);
        } else if (rx_data[0] == 0xFE) {
            i2c2_set_send_data((uint8_t *)&fm_version, 1);
        } else
            I2CRead(rx_data[0]);
    } else if (len > 1) {
        if ((rx_data[0] >= 0x50) && (rx_data[0] <= 0x52) &&
            ((len - 1 == 3) || (len - 1 == 4))) {
            rgb_color = rx_data[1] | (rx_data[2] << 8) | (rx_data[3] << 16);
            flag_rgb_color = 1;
        } else if ((rx_data[0] == 0x1C) && (len - 1 == 1)) {
            return;
        } else if ((rx_data[0] == 0x1D) && (len - 1 == 1)) {
            return;
        } else if ((rx_data[0] >= 0x1C) && (rx_data[0] <= 0x1D) &&
                   (len - 1 == 2)) {
            return;
        } else if ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x33)) {
            for (int i = 0; i < len - 1; i++) {
                g_scale_set[rx_data[0] - 0x30 + i] = rx_data[1 + i];
            }
            if ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x31)) {
                flag_scale_set_0 = 1;
                g_scale_set_0    = g_scale_set[0] | (g_scale_set[1] << 8);
            } else if ((rx_data[0] >= 0x32) && (rx_data[0] <= 0x33)) {
                flag_scale_set_load = 1;
                g_scale_set_load    = g_scale_set[2] | (g_scale_set[3] << 8);
            }
        } else if (rx_data[0] == 0xFF) {
            if (len == 2) {
                if ((rx_data[1] >= 0) && (rx_data[1] < 128)) {
                    if (readPackedMessageFromFlash(flash_arg, 8)) {
                        flash_arg[0] = rx_data[1];
                        flash_arg[1] = 0;
                        write_to_flash();
                        user_i2c_init();
                    }
                }
            }
        } else
            I2CWrite(rx_data[0], &rx_data[1], len - 1);
    }
    /* NOTE : This function should not be modified, when the callback is needed,
              the i2c2_receive_callback could be implemented in the user file
     */
}

__weak void i2c2_addr_req_callback(uint8_t TransferDirection) {
    UNUSED(TransferDirection);
}

void i2c2_set_send_data(uint8_t *tx_ptr, uint16_t len) {
    if (len > I2C_RECEIVE_BUFFER_LEN) {
        len = I2C_RECEIVE_BUFFER_LEN;
    }
    tx_len = len;

    if (len == 0 || tx_ptr == NULL) {
        return;
    }
    memcpy((void *)tx_buffer, tx_ptr, len);
}

void I2CInit(void) {
    HAL_I2C_EnableListen_IT(&hi2c1);
}

void I2CAddReg(uint8_t reg, uint8_t *buff, uint8_t len, uint8_t bit) {
    I2CReg_t *end = reg_list;

    if (end != NULL) {
        while (end->next != NULL) {
            end = end->next;
        }
        end->next = (I2CReg_t *)malloc(sizeof(I2CReg_t));
        end       = end->next;
    } else {
        end      = (I2CReg_t *)malloc(sizeof(I2CReg_t));
        reg_list = end;
    }

    end->next      = NULL;
    end->address   = reg;
    end->user_buff = buff;
    end->bit       = bit;
    end->len       = len;
    end->i2c_buff  = (uint8_t *)malloc(len);
    memcpy(end->i2c_buff, end->user_buff, end->len);
}

uint8_t I2CGetTxState() {
    return tx_state;
}

static void I2CRead(uint8_t reg) {
    I2CReg_t *list_ptr = reg_list;
    uint8_t offset     = reg & 0x0f;
    uint8_t num        = 0;
    uint8_t len        = 0;
    reg                = reg & 0xf0;

    while (list_ptr != NULL && list_ptr->address != reg) {
        list_ptr = list_ptr->next;
    }

    if (list_ptr == NULL) {
        i2c2_set_send_data(NULL, 0);
        return;
    }

    len = (offset > list_ptr->len) ? 0 : (list_ptr->len - offset);

    if (list_ptr->bit == 16) {
        num = list_ptr->len / 2;
        for (uint8_t i = 0; i < num; i++) {
            list_ptr->i2c_buff[2 * i]     = list_ptr->user_buff[i * 2 + 1];
            list_ptr->i2c_buff[2 * i + 1] = list_ptr->user_buff[i * 2];
        }
    } else if (list_ptr->bit == 32) {
        num = list_ptr->len / 4;
        for (uint8_t i = 0; i < num; i++) {
            list_ptr->i2c_buff[4 * i]     = list_ptr->user_buff[4 * i + 3];
            list_ptr->i2c_buff[4 * i + 1] = list_ptr->user_buff[4 * i + 2];
            list_ptr->i2c_buff[4 * i + 2] = list_ptr->user_buff[4 * i + 1];
            list_ptr->i2c_buff[4 * i + 3] = list_ptr->user_buff[4 * i];
        }
    } else {
        memcpy(list_ptr->i2c_buff, list_ptr->user_buff, list_ptr->len);
    }

    i2c2_set_send_data(&list_ptr->i2c_buff[offset], len);
}

static void I2CWrite(uint8_t reg, uint8_t *data, uint8_t len) {
    I2CReg_t *list_ptr = reg_list;
    uint8_t offset     = reg & 0x0f;
    uint8_t num        = 0;
    reg                = reg & 0xf0;

    while (list_ptr != NULL && list_ptr->address != reg) {
        list_ptr = list_ptr->next;
    }

    if (list_ptr == NULL || list_ptr->len < offset) {
        return;
    }

    len = ((list_ptr->len - offset) < len) ? (list_ptr->len - len) : len;

    if (list_ptr->bit == 16) {
        num = list_ptr->len / 2;
        for (uint8_t i = 0; i < num; i++) {
            list_ptr->i2c_buff[2 * i]     = list_ptr->user_buff[i * 2 + 1];
            list_ptr->i2c_buff[2 * i + 1] = list_ptr->user_buff[i * 2];
        }
    } else if (list_ptr->bit == 32) {
        num = list_ptr->len / 4;
        for (uint8_t i = 0; i < num; i++) {
            list_ptr->i2c_buff[4 * i]     = list_ptr->user_buff[4 * i + 3];
            list_ptr->i2c_buff[4 * i + 1] = list_ptr->user_buff[4 * i + 2];
            list_ptr->i2c_buff[4 * i + 2] = list_ptr->user_buff[4 * i + 1];
            list_ptr->i2c_buff[4 * i + 3] = list_ptr->user_buff[4 * i];
        }
    } else {
        memcpy(list_ptr->i2c_buff, list_ptr->user_buff, list_ptr->len);
    }

    memcpy(&list_ptr->i2c_buff[offset], data, len);

    if (list_ptr->bit == 16) {
        num = list_ptr->len / 2;
        for (uint8_t i = 0; i < num; i++) {
            *(uint16_t *)&list_ptr->user_buff[i * 2] =
                (list_ptr->i2c_buff[2 * i] << 8) |
                list_ptr->i2c_buff[2 * i + 1];
        }
    } else if (list_ptr->bit == 32) {
        num = list_ptr->len / 4;
        for (uint8_t i = 0; i < num; i++) {
            *(uint32_t *)&list_ptr->user_buff[i * 4] =
                (list_ptr->i2c_buff[4 * i] << 24) |
                (list_ptr->i2c_buff[4 * i + 1] << 16) |
                (list_ptr->i2c_buff[4 * i + 2] << 8) |
                (list_ptr->i2c_buff[4 * i + 3]);
        }
    } else {
        memcpy(list_ptr->user_buff, list_ptr->i2c_buff, list_ptr->len);
    }
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
                          uint16_t AddrMatchCode) {
    if (hi2c->Instance == hi2c1.Instance) {
        hi2c->State = HAL_I2C_STATE_READY;
        i2c2_addr_req_callback(TransferDirection);
        if (TransferDirection == I2C_WRITE_OPERATION) {
            HAL_I2C_Slave_Receive_IT(hi2c, (uint8_t *)rx_buffer,
                                     I2C_RECEIVE_BUFFER_LEN);
        } else {
            HAL_I2C_Slave_Transmit_IT(hi2c, (uint8_t *)tx_buffer, tx_len);
            tx_state = 1;
        }
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
        HAL_I2C_EnableListen_IT(&hi2c1);
    }
}

// read finish will callback
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c1.Instance) {
        if (tx_state != 1) {
            i2c2_receive_callback((uint8_t *)&rx_buffer[0],
                                  I2C_RECEIVE_BUFFER_LEN - hi2c->XferSize);
        }
        tx_state = 0;
        HAL_I2C_EnableListen_IT(&hi2c1);
    }
}

// write finish will callback
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c1.Instance) {
        i2c2_receive_callback((uint8_t *)&rx_buffer[0], I2C_RECEIVE_BUFFER_LEN);
        HAL_I2C_EnableListen_IT(&hi2c1);
    }
}

// write finish will callback
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c1.Instance) {
        tx_state = 0;
        HAL_I2C_EnableListen_IT(&hi2c1);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c1.Instance) {
        HAL_I2C_EnableListen_IT(&hi2c1);
        __HAL_I2C_GENERATE_NACK(&hi2c1);
    }
}
