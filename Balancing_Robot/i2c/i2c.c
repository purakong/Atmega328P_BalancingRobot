#include <inttypes.h>
#include <compat/twi.h>
#include <util/delay.h>

#include "i2c.h"
#include "../uart/uart.h"


#define F_CPU 16000000UL
// I2C clock in Hz
#define SCL_CLOCK  400000L



// Initialization of the I2C bus interface. Need to be called only once
void i2c_init(void)
{
  //Presacler is none 
  TWSR = 0;                       
  
  //I2c Clk is 400Khz  
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;

}


/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t twst;

	// send START condition
	//TWI Interrupt Flag(TWINT) is cleared by set one
	//If you want a data on the bus, you shoule write TWI START Condition Bit(TWSTA) as a one.
	//TWEN bit enables TWI operation and the TWI Interface
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	//TWINT bit is set by hardware when the TWI has finished
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) 
	{
		return 1;
	}

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) )
	{
		return 1;
	}
	
	return 0;

}/* i2c_start */

void i2c_start_check(uint8_t result)
{
	if(result == 1)
	{
		//I2C Init Failed
		UART_printString("I2C Initialization has been Failed");
		UART_transmit(10); UART_transmit(10);
	}
	else
	{
		//I2C Init Success
		PORTB |= _BV(5);
		_delay_ms(200);
		PORTB &= ~(_BV(5));
		_delay_ms(200);
		
		UART_printString("I2C Initialization has been successed");
		UART_transmit(10); UART_transmit(10);
	}
}


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );

}/* i2c_rep_start */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));

}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{	
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    

    return TWDR;

}/* i2c_readAck */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
    return TWDR;

}/* i2c_readNak */





// read one byte from dev, stored in value, return 1 for error
void i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data){

	i2c_start_wait(dev_addr+I2C_WRITE); 	//start i2c to write register address
	i2c_write(reg_addr);			//write address of register to read
	i2c_rep_start(dev_addr+I2C_READ);	//restart i2c to start reading
	*data = i2c_readNak();
        i2c_stop();

}



// write one byte to dev
void i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data){
	i2c_start_wait(dev_addr+I2C_WRITE);
 	i2c_write(reg_addr);                     
        i2c_write(data);
	i2c_stop();
}


// read multiple bytes from dev
void i2c_read_bytes(uint8_t dev_addr, uint8_t first_reg_addr, uint8_t length, uint8_t* data){
	i2c_start_wait(dev_addr+I2C_WRITE); 	//start i2c to write register address
	i2c_write(first_reg_addr);		//write address of register to read
	i2c_rep_start(dev_addr+I2C_READ);	//restart i2c to start reading

	uint8_t i;
	for(i=0; i<length-1; i++){
		*(data+i) = i2c_readAck();
	}
	*(data+i) = i2c_readNak();
        i2c_stop();
}



// write multiple bytes to dev
void i2c_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t* data){

	i2c_start_wait(dev_addr+I2C_WRITE);
	i2c_write(reg_addr);
	
	uint8_t i;
	for(i=0; i<length; i++){
		i2c_write( data[i] );
	}
	
	i2c_stop();

}




