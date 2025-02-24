/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Feb 6, 2025
 *      Author: omery
 */

#include "stm32f411xx.h"
#include <stdio.h>
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle );

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx -> CR1 |= (1 << I2C_CR1_START);
}


static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1 ;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr ;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1 ;
	SlaveAddr |= 1 ;
	pI2Cx->DR = SlaveAddr ;
}



static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}


}
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx -> CR1 |= (1 << I2C_CR1_STOP);
}


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
			if(pI2Cx == I2C1){
				I2C1_PCLK_EN();
			}else if(pI2Cx == I2C2){
				I2C2_PCLK_EN();
			}else if(pI2Cx == I2C3){
				I2C3_PCLK_EN();
			}
		}else{
			if(pI2Cx == I2C1){
				I2C1_PCLK_DI();
			}else if(pI2Cx == I2C2){
				I2C2_PCLK_DI();
			}else if(pI2Cx == I2C3){
				I2C3_PCLK_DI();
			}
			}

}
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

   //program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}


void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if (pI2Cx == I2C1) {
		I2C1_REG_RST();
	}else if (pI2Cx == I2C2) {
		I2C2_REG_RST();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RST();
	}
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB) );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until len becomes 0

	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) );

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF) );
	//burada loopta kalıyor.


	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}


void I2C_MasterReciveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr){

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB) );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);
	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));


	if(Len == 1){
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

		//generate STOP condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR ){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;



	}
	if(Len>1){	//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				if(pI2CHandle->Sr == I2C_DISABLE_SR ){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
			}
			}
	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
				}



uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState ;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
		pI2CHandle -> pTxBuffer = pTxBuffer;
		pI2CHandle -> TxLen = Len ;
		pI2CHandle -> TxRxState = I2C_BUSY_IN_TX ;
		pI2CHandle -> DevAddr = SlaveAddr ;
		pI2CHandle -> Sr = Sr ;


		//start
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//interrupt control enable
		//itbufen control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);


	}
	return busystate ;


}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr ;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx) ;

		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){

	if(pI2CHandle->TxLen > 0 ){
		//1. load the data in dr
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen-- ;

		//3.increöemt the buffer adress
		pI2CHandle->pTxBuffer++ ;

}
}




static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->RxSize == 1){

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR ;
		pI2CHandle->RxLen-- ;

	}
	if (pI2CHandle->RxSize > 1){
		if(pI2CHandle->RxLen ==2){

			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

	    *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;

	}
	if(pI2CHandle->RxLen == 0){
		//close the data reception and nıtify the app
		if(pI2CHandle->Sr == I2C_DISABLE_SR){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

	 I2C_CloseRecieveData(pI2CHandle);
	 I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

	}

}

void I2C_CloseRecieveData(I2C_Handle_t *pI2CHandle ){
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);



}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle ){
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;


}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//inyerrupt handling both master and slave

	uint32_t temp1,temp2,temp3 ;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	//1.handle for interrupt generetad by SB event
	// note: sb flag is only applicable in master mode
	if(temp1 && temp3){
		//sb flag is set
		//THİS BLOCK WONT BE EXECUTED İN SLAVE MODE SB İS ALWAYS ZERO
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3){
		//addr flag is set
		//do not forgot to clear!!
		I2C_ClearADDRFlag(pI2CHandle);


	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3){
		//btf flag is set

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){



		//make sure that txe is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){
			//btf and txe == 1;
				if(pI2CHandle->TxLen == 0){
			//1.generete stop cond.
			//2.reset all the member elements of the handle sturcture
					if(pI2CHandle->Sr == I2C_DISABLE_SR){
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
						I2C_CloseSendData(pI2CHandle);
			//closing i2c
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
			}
			}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

			;

		}





	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//stopf flag is not set in master mode
	//slave mode!! (stop condition)

	if(temp1 && temp3){
		//stopf flag is set
		//clear the stopf read sr1 then write to cr1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//notify (stop is genereted by master)
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}


	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp3 && temp2){
	//txe flag is set
	//DATA REG EMPTY
		//Master mode
		if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL)){

			//DO DATA TRANSMİSSİON
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);



			}
			}


	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp3 && temp2){
		//RXME flag is set
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				//do the data reception
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

		}
		}

}
}





void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag

		pI2CHandle->pI2Cx->SR1 &= (1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);


	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}


/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_ERROR_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}





void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}




void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );


}


void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	 if(EnorDi == ENABLE)
	 {
			pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
			pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
			pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	 }else
	 {
			pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
			pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
			pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	 }

}


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
  if(EnOrDi == ENABLE) {
    pI2Cx->CR1 |= (1 << I2C_CR1_PE);
  } else {
    pI2Cx->CR1 &= ~(1 << 0);
  }
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){

	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET ;
	}
	return FLAG_RESET ;

}



