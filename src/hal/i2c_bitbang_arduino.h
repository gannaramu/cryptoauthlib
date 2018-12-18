/**
 * \file
 * \brief  definitions for bit-banged I2C for Arduino
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

#ifndef I2C_BITBANG_ARDUINO_H_
#define I2C_BITBANG_ARDUINO_H_

#include "Arduino.h"
#include "atca_status.h"

#define MAX_I2C_BUSES   1

typedef struct
{
    uint8_t pin_sda[MAX_I2C_BUSES];
    uint8_t pin_scl[MAX_I2C_BUSES];
} I2CBuses;

extern I2CBuses i2c_buses_default;

extern uint8_t pin_sda;
extern uint8_t pin_scl;

#define I2C_ENABLE()            { pinMode(pin_sda, OUTPUT); pinMode(pin_scl, OUTPUT); }
#define I2C_DISABLE()           { pinMode(pin_sda, INPUT_PULLUP); pinMode(pin_scl, INPUT_PULLUP); }

#define I2C_CLOCK_LOW()         digitalWrite(pin_scl, LOW)
#define I2C_CLOCK_HIGH()        digitalWrite(pin_scl, HIGH)
#define I2C_DATA_LOW()          digitalWrite(pin_sda, LOW)
#define I2C_DATA_HIGH()         digitalWrite(pin_sda, HIGH)
#define I2C_DATA_IN()           digitalRead(pin_sda)
#define I2C_SET_OUTPUT()        pinMode(pin_sda, OUTPUT)
#define I2C_SET_OUTPUT_HIGH()   { I2C_SET_OUTPUT(); I2C_DATA_HIGH(); }
#define I2C_SET_OUTPUT_LOW()    { I2C_SET_OUTPUT(); I2C_DATA_LOW(); }
#define I2C_SET_INPUT()         pinMode(pin_sda, INPUT_PULLUP)
#define DISABLE_INTERRUPT()     noInterrupts()
#define ENABLE_INTERRUPT()      interrupts()


#define I2C_CLOCK_DELAY_WRITE_LOW()  delayMicroseconds(1)
#define I2C_CLOCK_DELAY_WRITE_HIGH() delayMicroseconds(1)
#define I2C_CLOCK_DELAY_READ_LOW()   delayMicroseconds(1)
#define I2C_CLOCK_DELAY_READ_HIGH()  delayMicroseconds(1)
#define I2C_CLOCK_DELAY_SEND_ACK()   delayMicroseconds(1)
//! This delay is inserted to make the Start and Stop hold time at least 250 ns.
#define I2C_HOLD_DELAY()             delayMicroseconds(1)


//! loop count when waiting for an acknowledgment
#define I2C_ACK_TIMEOUT                 (4)


/**
 * \brief Set I2C data and clock pin.
 *        Other functions will use these pins.
 *
 * \param[in] sda  definition of GPIO pin to be used as data pin
 * \param[in] scl  definition of GPIO pin to be used as clock pin
 */
void i2c_set_pin(uint8_t sda, uint8_t scl);

/**
 * \brief  Assigns the logical bus number for discovering the devices
 *
 *
 * \param[in]  i2c_bitbang_buses         The logical bus numbers are assigned to the variables.
 * \param[in]  max_buses                 Maximum number of bus used for discovering.
 */

void i2c_discover_buses(int i2c_bitbang_buses[], int max_buses);

/**
 * \brief Configure GPIO pins for I2C clock and data as output.
 */
void i2c_enable(void);

/**
 * \brief Configure GPIO pins for I2C clock and data as input.
 */
void i2c_disable(void);


/**
 * \brief Send a START condition.
 */
void i2c_send_start(void);

/**
 * \brief Send an ACK or NACK (after receive).
 *
 * \param[in] ack  0: NACK, else: ACK
 */
void i2c_send_ack(uint8_t ack);

/**
 * \brief Send a STOP condition.
 */
void i2c_send_stop(void);

/**
 * \brief Send a Wake Token.
 */
void i2c_send_wake_token(void);

/**
 * \brief Send one byte.
 *
 * \param[in] i2c_byte  byte to write
 *
 * \return ATCA_STATUS
 */
ATCA_STATUS i2c_send_byte(uint8_t i2c_byte);

/**
 * \brief Send a number of bytes.
 *
 * \param[in] count  number of bytes to send
 * \param[in] data   pointer to buffer containing bytes to send
 *
 * \return ATCA_STATUS
 */
ATCA_STATUS i2c_send_bytes(uint8_t count, uint8_t *data);

/**
 * \brief Receive one byte (MSB first).
 *
 * \param[in] ack  0:NACK, else:ACK
 *
 * \return Number of bytes received
 */
uint8_t i2c_receive_one_byte(uint8_t ack);

/**
 * \brief Receive one byte and send ACK.
 *
 * \param[out] data  pointer to received byte
 */
void i2c_receive_byte(uint8_t *data);

/**
 * \brief Receive a number of bytes.
 *
 * \param[out] data   pointer to receive buffer
 * \param[in]  count  number of bytes to receive
 */
void i2c_receive_bytes(uint8_t count, uint8_t *data);

#endif /* I2C_BITBANG_ARDUINO_H_ */