#include <MKL25Z4.H>
#include "MKL25z_I2C.h"


/*------------------------------------------------------------------------------
  Send an I2C start condition on the bus
 *------------------------------------------------------------------------------*/
int i2c_start() {
    // if we are in the middle of a transaction
    // activate the repeat_start flag
    if (I2C0->S & I2C_S_BUSY_MASK) {  
        I2C0->C1 |= 0x04;
    } else {
        I2C0->C1 |= I2C_C1_MST_MASK;
        I2C0->C1 |= I2C_C1_TX_MASK;
    }
    return 0;
}

/*------------------------------------------------------------------------------
  Send an I2C stop condition on the bus
 *------------------------------------------------------------------------------*/
void i2c_stop() {
    volatile uint32_t n = 0;
    I2C0->C1 &= ~I2C_C1_MST_MASK;
    I2C0->C1 &= ~I2C_C1_TX_MASK;

    // It seems that there are timing problems
    // when there is no waiting time after a STOP.
    // This wait is also included on the samples
    // code provided with the freedom board
    for (n = 0; n < 100; n++) __NOP();
}

/*------------------------------------------------------------------------------
  Wait for the end of the transmition on the I2C bus
 *------------------------------------------------------------------------------*/
int i2c_wait_end_tx_transfer() {
    // wait for the end of the tx transfer
    while((I2C0->S & I2C_S_IICIF_MASK) == 0);
    I2C0->S |= I2C_S_IICIF_MASK;

    // check if we received the ACK or not
    return I2C0->S & I2C_S_RXAK_MASK ? 1 : 0;
}

/*------------------------------------------------------------------------------
  Wait till the end of a recieve transmition on the bus
 *------------------------------------------------------------------------------*/
int i2c_wait_end_rx_transfer() {
    // wait for the end of the rx transfer
    while((I2C0->S & I2C_S_IICIF_MASK) == 0);
    I2C0->S |= I2C_S_IICIF_MASK;
    return 0;
}

/*------------------------------------------------------------------------------
  Send a nack condition on the bus
 *------------------------------------------------------------------------------*/
void i2c_send_nack() {
    I2C0->C1 |= I2C_C1_TXAK_MASK; // NACK
}

/*------------------------------------------------------------------------------
  Send an ack condition on the bus
 *------------------------------------------------------------------------------*/
void i2c_send_ack() {
    I2C0->C1 &= ~I2C_C1_TXAK_MASK; // ACK
}

/*------------------------------------------------------------------------------
  Write a value to the bus
 *------------------------------------------------------------------------------*/
int i2c_do_write(int value) {
    // write the data
    I2C0->D = value;

    // init and wait the end of the transfer
    return i2c_wait_end_tx_transfer();
}

/*------------------------------------------------------------------------------
  Read data from the bus 
 *------------------------------------------------------------------------------*/
int i2c_do_read(char * data, int last) {
    if (last)
        i2c_send_nack();
    else
        i2c_send_ack();

    *data = (I2C0->D & 0xFF);

    // start rx transfer and wait the end of the transfer
    return i2c_wait_end_rx_transfer();
}

/*------------------------------------------------------------------------------
  Configure the I2C bus
 *------------------------------------------------------------------------------*/
void I2C_configure(void)
{
	SIM->SCGC4 |= (1UL << 6);
	SIM->SCGC5 |= (1UL << 11);
		
	I2C0->F = ((0x0 << 6) | 0x00); //SET TO ZERO DUE TO ERRATA e6070
	I2C0->C1 = (1UL << 7);

}

/*------------------------------------------------------------------------------
  Send a character string of length to address
 *------------------------------------------------------------------------------*/
void I2C_send(int address, char* data, int length, int stop)
{
	int i;
	if (i2c_start()) {
        i2c_stop();
    }

    if (i2c_do_write((address & 0xFE) )) {
        i2c_stop();
    }

    for (i=0; i<length; i++) {
        if(i2c_do_write(data[i])) {
            i2c_stop();
        }
    }

		if(stop) {
        i2c_stop(); 
		}
}

/*------------------------------------------------------------------------------
  Read a string of length from address
 *------------------------------------------------------------------------------*/
void I2C_read(char address, char* data, int length, int stop)
{
	 int count;
    char * ptr;
    char dummy_read;

    if (i2c_start()) {
        i2c_stop();
    }

    if (i2c_do_write((address | 0x01))) {
        i2c_stop();
    }

    // set rx mode
    I2C0->C1 &= ~I2C_C1_TX_MASK;

    // Read in all except last byte
    for (count = 0; count < (length - 1); count++) {
        ptr = (count == 0) ? &dummy_read : &data[count - 1];
        if (i2c_do_read(ptr, 0)) {
            i2c_stop();
        }
    }

    // read in last byte
    ptr = (count == 0) ? &dummy_read : &data[count - 1];
    if (i2c_do_read(ptr, 1)) {
        i2c_stop();
    }

    // If not repeated start, send stop.
      if(stop) {
				i2c_stop();
			}

    // last read
    data[count] = I2C0->D;
}

/*------------------------------------------------------------------------------
  Power down the I2C module 
 *------------------------------------------------------------------------------*/
void I2C_powerDown(void)
{
		SIM->SCGC4 &= ~(1UL << 6);
	
}
