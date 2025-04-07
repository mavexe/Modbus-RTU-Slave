#include "i2c.h"
#include "SEGGER_RTT.h"
//#include "driver_init.h"

#define F_GCLK			48000000			//48MHz
#define F_SCL_STD		100000				//100khz	- w/ STD or FastMode+ = 20khz
#define F_SCL_FAST		400000				//400khz - w/ STD or FastMode+ = 76khz
#define F_SCL_FASTPLUS	1000000				//1Mhz - w/ STANDARD_MODE_FAST_MODE = 243khz
#define F_SCL_HS		3400000				//3.4MHz
uint32_t calculate_baud(uint32_t fgclk, uint32_t fscl)
{
	float f_baud;
	f_baud = (((float)fgclk/(float)fscl) - 10 - ((float)fgclk*0.0000003))/2;

	return ((uint32_t)f_baud);
}

I2C::I2C(Sercom* s, uint32_t pinSDA, uint32_t pinSCL, bool _on_irq)
{
  this->sercom = s;
  	on_irq=_on_irq;
  	pin_set_peripheral_function(pinSDA);
	pin_set_peripheral_function(pinSCL);
	timeout_master=20000;
	timeout_slave=5000;
	initClockNVIC();
}


void I2C::pin_set_peripheral_function(uint32_t pinmux)
{
	uint8_t port = (uint8_t)((pinmux >> 16)/32);
	PORT->Group[port].PINCFG[((pinmux >> 16) - (port*32))].bit.PMUXEN = 1;
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg &= ~(0xF << (4 * ((pinmux >> 16) & 0x01u)));
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg |= (uint8_t)((pinmux &0x0000FFFF) << (4 * ((pinmux >> 16) & 0x01u)));
}

void I2C::initClockNVIC()
{  
    uint32_t id = GCLK_CLKCTRL_ID_SERCOM0_CORE_Val;
    uint32_t apbMask = PM_APBCMASK_SERCOM0;
    uint32_t irqn = SERCOM0_IRQn;
	
	if( sercom == SERCOM0 ) {
        id = GCLK_CLKCTRL_ID_SERCOM0_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM0;
        irqn = SERCOM0_IRQn;
    }
	
    if( sercom == SERCOM1 ) {
        id = GCLK_CLKCTRL_ID_SERCOM1_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM1;
        irqn = SERCOM1_IRQn;
    }
    if( sercom == SERCOM2 ) {
        id = GCLK_CLKCTRL_ID_SERCOM2_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM2;
        irqn = SERCOM2_IRQn;
    }
    if( sercom == SERCOM3 ) {
        id = GCLK_CLKCTRL_ID_SERCOM3_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM3;
        irqn = SERCOM3_IRQn;
    }
    if( sercom == SERCOM4 ) {
        id = GCLK_CLKCTRL_ID_SERCOM4_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM4;
        irqn = SERCOM4_IRQn;
    }

    if( sercom == SERCOM5 ) {
        id = GCLK_CLKCTRL_ID_SERCOM5_CORE_Val;
        apbMask = PM_APBCMASK_SERCOM5;
        irqn = SERCOM5_IRQn;
    }

    // Ensure that PORT is enabled
    enableAPBBClk( PM_APBBMASK_PORT, 1 );
    initGenericClk( GCLK_CLKCTRL_GEN_GCLK0_Val, id );
    enableAPBCClk( apbMask, 1 );
    if(on_irq)NVIC_EnableIRQ( (IRQn_Type)irqn );  
}


void I2C::enableAPBBClk( uint32_t item, uint8_t enable )
{
    item &= PM_APBBMASK_MASK;
    if( enable )
        PM->APBBMASK.reg |= item;
    else
        PM->APBBMASK.reg &= ~( item );
}

void I2C::enableAPBCClk( uint32_t item, uint8_t enable )
{
    item &= PM_APBCMASK_MASK;
    if( enable )
        PM->APBCMASK.reg |= item;
    else
        PM->APBCMASK.reg &= ~( item );
}


int8_t I2C::initGenericClk( uint32_t genClk, uint32_t id )
{
	if( genClk > GCLK_CLKCTRL_GEN_GCLK7_Val ) return -1;
	if( id > GCLK_CLKCTRL_ID_PTC_Val ) return -1;	
	while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( id ) | GCLK_CLKCTRL_GEN( genClk ) |
	GCLK_CLKCTRL_CLKEN;
	return 0;
}



void I2C::setSlave(uint8_t address) {
 
  	slaveAddr=address;
	sercom->I2CM.CTRLA.bit.ENABLE = 0 ;
  	while (sercom->I2CM.STATUS.bit.SYNCBUSY){}

	sercom->I2CS.CTRLA.bit.MODE = 4;
	sercom->I2CS.ADDR.reg = SERCOM_I2CS_ADDR_ADDR( address & 0x7Ful ) | // 0x7F, select only 7 bits
	SERCOM_I2CS_ADDR_ADDRMASK( 0x00ul );          // 0x00, only match exact address
	
	// Set the interrupt register
	if(on_irq)
	{
		sercom->I2CS.INTENSET.reg = SERCOM_I2CS_INTENSET_PREC |   // Stop
		SERCOM_I2CS_INTENSET_AMATCH | // Address Match
		SERCOM_I2CS_INTENSET_DRDY ;   // Data Ready
	}
	//sercom->I2CS.CTRLA.reg = 2097170;
	
	sercom->I2CS.CTRLB.reg = SERCOM_I2CS_CTRLB_SMEN | SERCOM_I2CS_CTRLB_AMODE(0x0);
	
	while (sercom->I2CM.STATUS.bit.SYNCBUSY){}		
	sercom->I2CM.CTRLA.bit.ENABLE = 1;		
	while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
  	// Setting bus idle mode
 	sercom->I2CM.STATUS.bit.BUSSTATE = 1;
  	while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
}

int I2C::i2c_slave_wait_for_bus()
{	
        uint16_t timeout_counter = 0;
	//ожидание когда что-то придёт по линии i2c
  	while ((!(sercom->I2CS.INTFLAG.reg & SERCOM_I2CS_INTFLAG_DRDY)) &&
              (!(sercom->I2CS.INTFLAG.reg & SERCOM_I2CS_INTFLAG_PREC)) &&
              (!(sercom->I2CS.INTFLAG.reg & SERCOM_I2CS_INTFLAG_AMATCH))) {
		if (++timeout_counter >= timeout_slave) { return -10;}	
	}		
	return 0;
}

int I2C::i2c_master_wait_for_bus(){	
    uint16_t timeout_counter = 0;
    while (!(sercom->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB) &&
           !(sercom->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB)) {
            // Check timeout condition. 
             if (++timeout_counter >= timeout_master) { return -10;}
           }
     return 0;
}

int I2C::slave_get_direction_wait()
{  
	int state=i2c_slave_wait_for_bus();
	
	if(state!=0)
	{ return state;}
	
	if (!(sercom->I2CS.INTFLAG.reg & SERCOM_I2CS_INTFLAG_AMATCH)) {
		/* Not address interrupt, something is wrong */
		return 0;
	}
	/* Check direction */
	if ((sercom->I2CS.STATUS.reg & SERCOM_I2CS_STATUS_DIR)) {
		/* Read request from master */
		return 1;
	} else {
		/* Write request from master */
		return 2;
	}	
	//return rez;	
}

void I2C::i2c_slave_set_ctrlb_ackact(bool send_ack)
{
	sercom->I2CS.STATUS.reg = 0;
	if (send_ack == true) {
		sercom->I2CS.CTRLB.reg = 0;
	}
	else {
		sercom->I2CS.CTRLB.reg = SERCOM_I2CS_CTRLB_ACKACT;
	}
}

void I2C::i2c_slave_set_ctrlb_cmd3()
{
  if (sercom->I2CS.INTFLAG.bit.PREC) {
		sercom->I2CS.INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
	}
	sercom->I2CS.INTFLAG.reg = SERCOM_I2CS_INTFLAG_AMATCH;  
}

int I2C::slave_read_packet_wait(uint8_t *buf, uint8_t len_buf, uint8_t &len_in)
{  
  	if(isMaster())setSlave(slaveAddr);
  	int status=i2c_slave_wait_for_bus();
	if(status!=0)return status;
	
	if (!(sercom->I2CS.INTFLAG.reg & SERCOM_I2CS_INTFLAG_AMATCH)) {
		/* Not address interrupt, something is wrong */
		return -1;
	}

	/* Check if there was an error in the last transfer */
	if (sercom->I2CS.STATUS.reg & (SERCOM_I2CS_STATUS_BUSERR |
			SERCOM_I2CS_STATUS_COLL | SERCOM_I2CS_STATUS_LOWTOUT)) {
		return -2;
	}	
	
	if ((sercom->I2CS.STATUS.reg & SERCOM_I2CS_STATUS_DIR)) {
		// Write request from master, send NACK and return
		i2c_slave_set_ctrlb_ackact(false);
		i2c_slave_set_ctrlb_cmd3();
		return -1;
	}	
	i2c_slave_set_ctrlb_ackact(true);
	i2c_slave_set_ctrlb_cmd3();
		
	len_in = 0;
	uint16_t length = len_buf;
	while (length--) {
	  	status=i2c_slave_wait_for_bus();
		if (status != 0) {
			// Timeout, return
			return status;
		}		
		if ((sercom->I2CS.INTFLAG.reg & SERCOM_I2CS_INTFLAG_PREC) ||
				sercom->I2CS.INTFLAG.reg & SERCOM_I2CS_INTFLAG_AMATCH) {
			// Master sent stop condition, or repeated start, read done 
			// Clear stop flag 
			sercom->I2CS.INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
			return 0;
		}		
		
		// Read data 
		while (sercom->I2CS.STATUS.bit.SYNCBUSY){}
		buf[len_in++] = sercom->I2CS.DATA.reg;
	}
	i2c_slave_wait_for_bus();
	if (sercom->I2CS.INTFLAG.reg & SERCOM_I2CS_INTFLAG_DRDY) {
		// Buffer is full, send NACK 
		i2c_slave_set_ctrlb_ackact(false);
		sercom->I2CS.CTRLB.reg |= SERCOM_I2CS_CTRLB_CMD(0x2);
	}
	if (sercom->I2CS.INTFLAG.reg & SERCOM_I2CS_INTFLAG_PREC) {
		/* Clear stop flag */
		sercom->I2CS.INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
	}
	return 0;
}

int I2C::slave_write_packet_wait(uint8_t *buf, uint8_t len)
{
  	if(isMaster())setSlave(slaveAddr);
	i2c_slave_wait_for_bus();
	if (!(sercom->I2CS.STATUS.reg & SERCOM_I2CS_STATUS_DIR)) {
		// Write request from master, send NACK and return
		i2c_slave_set_ctrlb_ackact(false);
		i2c_slave_set_ctrlb_cmd3();
		return -1;
	}	
	i2c_slave_set_ctrlb_ackact(true);
	i2c_slave_set_ctrlb_cmd3();  
  	i2c_slave_wait_for_bus();
	uint16_t i = 0;
	uint16_t length = len;
	while (length--) {
	  	while (sercom->I2CS.STATUS.bit.SYNCBUSY){}
		sercom->I2CS.DATA.reg = buf[i++];
		i2c_slave_wait_for_bus();
		if (sercom->I2CS.STATUS.reg & SERCOM_I2CS_STATUS_RXNACK &&	length !=0) {
			// NACK from master, abort
			// Release line
			sercom->I2CS.CTRLB.reg |= SERCOM_I2CS_CTRLB_CMD(0x02);			
			return -2;
		}
	}
	sercom->I2CS.CTRLB.reg |= SERCOM_I2CS_CTRLB_CMD(0x02);	
	return 0;
}

int I2C::parseMasterWireStatus()
{
    // Check in descending order according to figure 26-5 in the data sheet
    int status = sercom->I2CM.STATUS.reg;

    // Bus busy
    if( ( status & SERCOM_I2CM_STATUS_BUSSTATE( 3 ) ) ==
        SERCOM_I2CM_STATUS_BUSSTATE( 3 ) ) {
        return -1;
    }

    // Error condition
    if( status & ( SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_ARBLOST ) )
        return -2;

    // Low time out
    if( status & SERCOM_I2CM_STATUS_LOWTOUT ) return -3;

    // RX NACK
    if( status & SERCOM_I2CM_STATUS_RXNACK ) return -4;

    return 0;
}

//проверка линии перед отправкой
int I2C::i2c_master_address_response()
{
	// Check for error and ignore bus-error; workaround for BUSSTATE stuck in BUSY 
	if (sercom->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB) {

		/* Clear write interrupt flag */
		sercom->I2CM.INTFLAG.reg = SERCOM_I2CM_INTFLAG_SB;

		/* Check arbitration. */
		if (sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_ARBLOST) {
			/* Return packet collision. */
			return -1;
		}
	/* Check that slave responded with ack. */
	} else if (sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK) {
		/* Slave busy. Issue ack and stop command. */
		sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);

		/* Return bad address value. */
		return -2;
	}

	return 0;
}


int I2C::setMaster() {
    int status = parseMasterWireStatus();
    if( status != -1 )   //если линия занята -не переключаемся на мастера
    {
		sercom->I2CM.CTRLA.bit.ENABLE = 0 ;
  		while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
		
		sercom->I2CM.CTRLA.bit.MODE = 5;		
		// Clear Stop interrupt 
		if(on_irq)
		{
			sercom->I2CS.INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
			// Disable interrupts 
			sercom->I2CS.INTENCLR.reg = SERCOM_I2CS_INTFLAG_PREC | SERCOM_I2CS_INTFLAG_DRDY;		
			//setClock();
			sercom->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(calculate_baud(F_GCLK, F_SCL_FAST)) | SERCOM_I2CM_BAUD_BAUDLOW(0);
		}
		while (sercom->I2CM.STATUS.bit.SYNCBUSY){}		
		sercom->I2CM.CTRLA.bit.ENABLE = 1;		
		while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
  		// Setting bus idle mode
 		sercom->I2CM.STATUS.bit.BUSSTATE = 1;
  		while (sercom->I2CM.STATUS.bit.SYNCBUSY){}		
			
		return 0;
  }	else return status;
}


int I2C::master_write_packet(uint8_t addr, uint8_t *buf, uint8_t len)
{
  	if(isSlave())setMaster();
    bool isWrite=true;
    while (sercom->I2CM.STATUS.bit.SYNCBUSY){}	
    addr <<= 0x01;
    if( !isWrite ) addr |= 0x01;
    sercom->I2CM.ADDR.bit.ADDR = addr;
    int status = i2c_master_wait_for_bus();    
    //int status = i2c_master_address_response();
    //if(status==0)status = parseMasterWireStatus();
	if(status==0)status = i2c_master_address_response();
    uint16_t length = len;
    uint16_t i=0;
    if(status==0)
    {	
        while (length--) 
        {
            if (!(sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_BUSSTATE(2))) {
                return -3;
            }
            // Write byte to slave. 
            while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
            sercom->I2CM.DATA.reg = buf[i++];
            // Wait for response. 
            status=i2c_master_wait_for_bus();
			if(status!=0)break;
            // Check for NACK from slave. 
            if (sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK) {
              // Return bad data value. 
                status = -5;
                break;
            }
        }		
        while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
        sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);  
    }
	//delay_ms(5);
    return status;  
    //return parseMasterWireStatus();
	//0	-нет ошибок
	//-2 нет линии
}

int I2C::master_read_packet(uint8_t addr, uint8_t *buf, uint8_t len)
{  	
  	if(isSlave())setMaster();
    addr <<= 0x01;
    addr |= 0x01;
    // Don't try anything if the bus is busy
    int status = parseMasterWireStatus();
    if( status == -1 || status == -2 )return status;
    // Send start and address    
     while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
        sercom->I2CM.ADDR.bit.ADDR = addr; 
	 	sercom->I2CM.CTRLB.bit.ACKACT = 0;//( true ? 0 : 1 );// Send an ACK
    	for( int i = 0; i < len; i++ ) {
		  	sercom->I2CM.CTRLB.bit.ACKACT = 0;                         // Prepare Acknowledge
			sercom->I2CM.CTRLB.bit.CMD = 2;
    	    // Wait for the bits to clear
    	    //if( waitForSBWire() != I2CM_ERR_NONE ) break;
			
		    while( !sercom->I2CM.INTFLAG.bit.SB ) {
    	   		// If the slave NACKS the address, the MB bit will be set
    	    	//if( sercom->sercom->I2CM.INTFLAG.bit.MB ) sercom->parseMasterWireStatus();
    		}		
	
    	    // Send a stop
    	    if(i == ( len - 1 )) {
    	        sercom->I2CM.CTRLB.bit.ACKACT = ( false ? 0 : 1 );
    	        //prepareMasterCommandWIRE( WIRE_MASTER_ACK_STOP );
				while(sercom->I2CM.STATUS.bit.SYNCBUSY){}
				sercom->I2CM.CTRLB.bit.CMD = 3;
    	    }		
			//while( sercom->I2CM.INTFLAG.bit.SB == 0 ){}		
    	    buf[i] = sercom->I2CM.DATA.bit.DATA;
			//SEGGER_RTT_printf(0, "%d",sercom->I2CM.DATA.bit.DATA);
    	}
		return parseMasterWireStatus();   
}
/*
int I2C::master_read_packet(uint8_t addr, uint8_t *buf, uint8_t len)
{
	bool isWrite=false;
	addr <<= 0x01;
	if( !isWrite ) addr |= 0x01;
	sercom->I2CM.ADDR.bit.ADDR = addr;
	
	int status=i2c_master_wait_for_bus();   

	// Set action to ack. 
while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
	sercom->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;

	// Check for address response error unless previous error is detected. 
	if (status == 0) {
		status = i2c_master_address_response();
	}
        uint16_t i=0;
        uint16_t length = len;
	// Check that no error has occurred. 
	if (status == 0) {
	  	
		// Read data buffer. 
		while (length--) {		  
		  	sercom->I2CM.CTRLB.bit.CMD = 2;
		  	sercom->I2CM.CTRLB.bit.ACKACT = 0;                         // Prepare Acknowledge
			// Check that bus ownership is not lost. 
			if (!(sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_BUSSTATE(2))) {
				return -3;
			}

			if (length == 0) {
				// Set action to NACK 
				sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT;
			} else {
				// Save data to buffer. 
				while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
				buf[i++] = sercom->I2CM.DATA.reg;
				// Wait for response. 
				status = i2c_master_wait_for_bus(); 
			}

			// Check for error. 
			if (status != 0) {
				break;
			}
		}
		
		// Send stop command unless arbitration is lost. 
		while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
		sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
		
		// Save last data to buffer. 
		while (sercom->I2CM.STATUS.bit.SYNCBUSY){}
		buf[i] = sercom->I2CM.DATA.reg;
	}
	
	return status;
}*/

int8_t I2C::masterReadBuf(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  	uint8_t result=master_write_packet(addr, &reg, 1);
	if(result==0)
	{
		result=master_read_packet(addr, buf, len);
	}
	return result;
}


int8_t I2C::masterReadByte(uint8_t addr, uint8_t &b, uint8_t reg)
{
  	uint8_t result=master_write_packet(addr, &reg, 1);
	if(result==0)
	{
		result=master_read_packet(addr, &b, 1);
	}
	return result;
}


int8_t I2C::masterReadWord(uint8_t addr, uint16_t &w, uint8_t reg)
{
  	uint8_t buf[2];
  	uint8_t result=master_write_packet(addr, &reg, 1);
	if(result==0)
	{
		result=master_read_packet(addr, buf, 2);
		w=(((uint16_t) buf[0]) << 8) | ((uint16_t) buf[1]);
	}	
	return result;
}



bool I2C::isMaster()
{
  	return sercom->I2CS.CTRLA.bit.MODE == 5;
}

bool I2C::isSlave()
{
  return sercom->I2CS.CTRLA.bit.MODE == 4;
}


void I2C::reset()
{
  //Setting the Software bit to 1
  sercom->I2CM.CTRLA.bit.SWRST = 1;
  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(sercom->I2CM.CTRLA.bit.SWRST || sercom->I2CM.STATUS.bit.SYNCBUSY){};
}


void I2C::onReceive(void(*function)(int))
{
  onReceiveCallback = function;
}

void I2C::onRequest(void(*function)(void))
{
  onRequestCallback = function;
}



//static int ir=0;
//static int it=0;
//обработка прерывания при приёме
void I2C::onService(void)
{  	  	
  	//stop
	if (sercom->I2CS.INTFLAG.bit.PREC)
	{ 	  	
	  	//sercom->I2CS.INTFLAG.bit.PREC = 1;
		sercom->I2CS.CTRLB.bit.ACKACT = 0;
		sercom->I2CS.CTRLB.bit.CMD = 0x3;
		
		if (sercom->I2CS.STATUS.bit.DIR==0 && onReceiveCallback)
		{
			available=ir;		
			onReceiveCallback(ir);
		}	
		if (sercom->I2CS.STATUS.bit.DIR==1)
		{
			
		}			
	}	
	//обработка адреса
	else if(sercom->I2CS.INTFLAG.bit.AMATCH)
	{	
		sercom->I2CS.INTFLAG.bit.AMATCH = 1;		
		it=0;		
      	if(sercom->I2CS.STATUS.bit.DIR==1 && onReceiveCallback)
      	{  
        	//onReceiveCallback(ir);
		  	onRequestCallback();
      	}	
		if(sercom->I2CS.STATUS.bit.DIR==0)
      	{  
		  	ir=0;
        	available=0;
			
      	}
		//SEGGER_RTT_printf(0, "*");
	}  
	//data ready
	else if(sercom->I2CS.INTFLAG.bit.DRDY)
	{			
	  	//приём
		if (sercom->I2CS.STATUS.bit.DIR==0)
		{ 		  	
		  	rx_buf[ir++] = sercom->I2CS.DATA.reg;			
			sercom->I2CS.CTRLB.bit.CMD = 0x3;
			sercom->I2CS.CTRLB.bit.ACKACT = 0;
		}
		//отправка на запрос
		else if (sercom->I2CS.STATUS.bit.DIR==1)
		{		  	 			  	
			sercom->I2CS.DATA.reg = tx_buf[it++];			
		}		
	}
  
}


