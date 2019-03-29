/**
 * \file
 * \brief  definitions for I2C for Arduino
 *
 * \copyright (c) 2018 Gabriel Notman.
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip software
 * and any derivatives exclusively with Microchip products. It is your
 * responsibility to comply with third party license terms applicable to your
 * use of third party software (including open source software) that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT,
 * SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE
 * OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
 * MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
 * FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL
 * LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED
 * THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR
 * THIS SOFTWARE.
 */

#include "Arduino.h"
#include "Wire.h"

#include "arduino_hal_select.h"
#include "atca_hal.h"
#include "atca_iface.h"
#include "atca_status.h"
#include "i2c_arduino.h"

#if defined(USE_ARDUINO_I2C)

extern "C" {

ATCA_STATUS hal_i2c_init( void *hal, ATCAIfaceCfg *cfg);
ATCA_STATUS hal_i2c_post_init(ATCAIface iface);
ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t *txdata, int txlength);
ATCA_STATUS hal_i2c_receive( ATCAIface iface, uint8_t *rxdata, uint16_t *rxlength);
ATCA_STATUS hal_i2c_wake(ATCAIface iface);
ATCA_STATUS hal_i2c_idle(ATCAIface iface);
ATCA_STATUS hal_i2c_sleep(ATCAIface iface);
ATCA_STATUS hal_i2c_release(void *hal_data );
ATCA_STATUS hal_i2c_discover_buses(int i2c_buses[], int max_buses);
ATCA_STATUS hal_i2c_discover_devices(int busNum, ATCAIfaceCfg *cfg, int *found );

};

ATCA_STATUS hal_i2c_init(void *hal, ATCAIfaceCfg *cfg)
{
    if (cfg->iface.atcai2c.bus >= MAX_I2C_BUSES) {
        return ATCA_COMM_FAIL;
    }
    
    Wire.begin();
    Wire.setClock(cfg->iface.atcai2c.baud);
    
    return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_post_init(ATCAIface iface)
{
  return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t *txdata, int txlength)
{
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    
    txdata[0] = 0x03;
    txlength++;
    
    ATCA_STATUS result = ATCA_SUCCESS;
    
    Wire.beginTransmission(cfg->iface.atcai2c.slave_address >> 1);
    if (Wire.write(txdata,txlength) != txlength) {
        result = ATCA_COMM_FAIL;
    }
    Wire.endTransmission();

  return result;
}

ATCA_STATUS hal_i2c_receive( ATCAIface iface, uint8_t *rxdata, uint16_t *rxlength)
{
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    int retries = cfg->rx_retries;
    int status = !ATCA_SUCCESS;
    uint16_t rxdata_max_size = *rxlength;
    
    *rxlength = 0;
        
    if (rxdata_max_size < 1) {
        return ATCA_SMALL_BUFFER;
    }
    
    while (retries-- > 0 && status != ATCA_SUCCESS) {
        if (Wire.requestFrom(cfg->iface.atcai2c.slave_address >> 1, 1) != 1) {
            status = ATCA_COMM_FAIL;
        }
        else {
            rxdata[0] = Wire.read();
            status = ATCA_SUCCESS;
        }
    }
    
    if (status != ATCA_SUCCESS) {
        return (ATCA_STATUS)status;
    }
    if (rxdata[0] < ATCA_RSP_SIZE_MIN) {
        return ATCA_INVALID_SIZE;
    }
    if (rxdata[0] > rxdata_max_size) {
        return ATCA_SMALL_BUFFER;
    }
    
    uint8_t len = rxdata[0] - 1;
    if (Wire.requestFrom(cfg->iface.atcai2c.slave_address >> 1, len) != len) {
        status = ATCA_COMM_FAIL;
    }
    else {
        for (uint8_t i = 1; i <= len; i++) {
            rxdata[i] = Wire.read();
        }
        status = ATCA_SUCCESS;
    }
    
    if (status != ATCA_SUCCESS) {
        return (ATCA_STATUS)status;
    }

    *rxlength = rxdata[0];

    return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_wake(ATCAIface iface)
{
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    int retries = cfg->rx_retries;
    uint32_t bdrt = cfg->iface.atcai2c.baud;
    
    bool status = true;
    uint8_t data[4];
    memset(data, 0, sizeof(data));
    
    Wire.setClock(100000);
    
    Wire.beginTransmission(0x00);
    Wire.endTransmission(false);
    
    atca_delay_us(cfg->wake_delay);
    
    while (retries-- > 0 && status)
    {
        status = (Wire.requestFrom(cfg->iface.atcai2c.slave_address >> 1, sizeof(data)) == sizeof(data));
    }
    
    if (status) {
        for (uint8_t i = 0; i < sizeof(data); i++) {
            data[i] = Wire.read();
        }
        Wire.setClock(bdrt);
    }
    
    return hal_check_wake(data, 4);
}

ATCA_STATUS hal_i2c_idle(ATCAIface iface)
{
  ATCAIfaceCfg *cfg = atgetifacecfg(iface);
  Wire.beginTransmission(cfg->iface.atcai2c.slave_address >> 1);
  Wire.write(0x02); // idle word address value
  Wire.endTransmission();
  atca_delay_ms(1);
  return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_sleep(ATCAIface iface)
{
  ATCAIfaceCfg *cfg = atgetifacecfg(iface);
  Wire.beginTransmission(cfg->iface.atcai2c.slave_address >> 1);
  Wire.write(0x01); // sleep word address value
  Wire.endTransmission();
  atca_delay_ms(1);
  return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_release(void *hal_data )
{
  return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_discover_buses(int i2c_buses[], int max_buses)
{
  return ATCA_UNIMPLEMENTED;
}

ATCA_STATUS hal_i2c_discover_devices(int busNum, ATCAIfaceCfg *cfg, int *found )
{
  return ATCA_UNIMPLEMENTED;
}
#endif






