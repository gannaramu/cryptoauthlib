/**
 * \file
 * \brief  Hardware Interface Functions - I2C bit-bang for Arduino
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
#include "i2c_bitbang_arduino.h"

#define DEFAULT_I2C_BUS 0

uint8_t pin_sda, pin_scl;

I2CBuses i2c_buses_default = {
    { 2 },// Arduino defualt SDA header pin
    { 3 } // Arduino defualt SCL header pin
};

uint8_t pin_sda, pin_scl;

void i2c_discover_buses(int i2c_bitbang_buses[], int max_buses)
{
    i2c_bitbang_buses[0] = DEFAULT_I2C_BUS;
}

void i2c_set_pin(uint8_t sda, uint8_t scl)
{
    pin_sda = sda;
    pin_scl = scl;
}

void i2c_enable(void)
{
    I2C_ENABLE();
    I2C_DATA_HIGH();
    I2C_CLOCK_HIGH();
}

void i2c_disable(void)
{
    I2C_DISABLE();
}

void i2c_send_start(void)
{
    //! Set clock high in case we re-start.
    I2C_CLOCK_HIGH();
    I2C_SET_OUTPUT_HIGH();
    I2C_DATA_LOW();
    I2C_HOLD_DELAY();
    I2C_CLOCK_LOW();
}

void i2c_send_ack(uint8_t ack)
{
    if (ack)
    {
        I2C_SET_OUTPUT_LOW();   //!< Low data line indicates an ACK.
        while (I2C_DATA_IN())
        {
            ;
        }
    }
    else
    {
        I2C_SET_OUTPUT_HIGH();  //!< High data line indicates a NACK.
        while (!I2C_DATA_IN())
        {
            ;
        }
    }

    //! Clock out acknowledgment.
    I2C_CLOCK_HIGH();
    I2C_CLOCK_DELAY_SEND_ACK();
    I2C_CLOCK_LOW();
}

void i2c_send_stop(void)
{
    I2C_SET_OUTPUT_LOW();
    I2C_CLOCK_DELAY_WRITE_LOW();
    I2C_CLOCK_HIGH();
    I2C_HOLD_DELAY();
    I2C_DATA_HIGH();
}


void i2c_send_wake_token(void)
{
    I2C_DATA_LOW();
    atca_delay_us(80);
    I2C_DATA_HIGH();
}

ATCA_STATUS i2c_send_byte(uint8_t i2c_byte)
{
    ATCA_STATUS status = ATCA_TX_TIMEOUT;

    uint8_t i;

    DISABLE_INTERRUPT();

    //! This avoids spikes but adds an if condition.
    //! We could parametrize the call to I2C_SET_OUTPUT
    //! and translate the msb to OUTSET or OUTCLR,
    //! but then the code would become target specific.
    if (i2c_byte & 0x80)
    {
        I2C_SET_OUTPUT_HIGH();
    }
    else
    {
        I2C_SET_OUTPUT_LOW();
    }

    //! Send 8 bits of data.
    for (i = 0; i < 8; i++)
    {
        I2C_CLOCK_LOW();
        if (i2c_byte & 0x80)
        {
            I2C_DATA_HIGH();
        }
        else
        {
            I2C_DATA_LOW();
        }
        I2C_CLOCK_DELAY_WRITE_LOW();

        //! Clock out the data bit.
        I2C_CLOCK_HIGH();

        //! Shifting while clock is high compensates for the time it
        //! takes to evaluate the bit while clock is low.
        //! That way, the low and high time of the clock pin is
        //! almost equal.
        i2c_byte <<= 1;
        I2C_CLOCK_DELAY_WRITE_HIGH();
    }
    //! Clock in last data bit.
    I2C_CLOCK_LOW();

    //! Set data line to be an input.
    I2C_SET_INPUT();

    I2C_CLOCK_DELAY_READ_LOW();
    //! Wait for the ack.
    I2C_CLOCK_HIGH();
    for (i = 0; i < I2C_ACK_TIMEOUT; i++)
    {
        if (!I2C_DATA_IN())
        {
            status = ATCA_SUCCESS;
            I2C_CLOCK_DELAY_READ_HIGH();
            break;
        }
    }
    I2C_CLOCK_LOW();

    ENABLE_INTERRUPT();

    return status;
}

ATCA_STATUS i2c_send_bytes(uint8_t count, uint8_t *data)
{
    ATCA_STATUS status = ATCA_TX_TIMEOUT;

    uint8_t i;

    for (i = 0; i < count; i++)
    {
        status = i2c_send_byte(data[i]);
        if (status != ATCA_SUCCESS)
        {
            if (i > 0)
            {
                status = ATCA_TX_FAIL;
            }
            break;
        }
    }

    return status;
}

uint8_t i2c_receive_one_byte(uint8_t ack)
{
    uint8_t i2c_byte;
    uint8_t i;

    DISABLE_INTERRUPT();

    I2C_SET_INPUT();
    for (i = 0x80, i2c_byte = 0; i; i >>= 1)
    {
        I2C_CLOCK_HIGH();
        I2C_CLOCK_DELAY_READ_HIGH();
        if (I2C_DATA_IN())
        {
            i2c_byte |= i;
        }
        I2C_CLOCK_LOW();
        if (i > 1)
        {
            //! We don't need to delay after the last bit because
            //! it takes time to switch the pin to output for acknowledging.
            I2C_CLOCK_DELAY_READ_LOW();
        }
    }
    i2c_send_ack(ack);

    ENABLE_INTERRUPT();

    return i2c_byte;
}

void i2c_receive_byte(uint8_t *data)
{
    *data = i2c_receive_one_byte(1);
}

void i2c_receive_bytes(uint8_t count, uint8_t *data)
{
    while (--count)
    {
        *data++ = i2c_receive_one_byte(1);
    }
    *data = i2c_receive_one_byte(0);

    i2c_send_stop();
}