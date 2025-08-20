/*
 * MMC5983.c
 *
 * Created on: Jul 29, 2025
 * Author: Nathan Netzel
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Nathan Netzel
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the conditions of the MIT License.
 */


#include <MMC5983.h>


/* Private SPI Handler */



/**
 * @brief  Read the MMC ID register and checks its content
 * @note   Returns MMC_NO_ERROR only if content matches the expected value
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @retval MMC5983_Error_TypeDef
 */
MMC5983_Error_TypeDef MMC5983_ID_Verify(MMC5983_HW_InitTypeDef *MMC5983_Handler){
	uint8_t ID_Receive=0;

	MMC5983_Error_TypeDef Verify_Status = MMC5983_SingleRegister_Read(MMC5983_Handler, MMC_ID_REG, &ID_Receive);
	if ( (ID_Receive==MMC_DEVICE_ID) && (Verify_Status==MMC_NO_ERROR) )
		return MMC_NO_ERROR;
	else
		return MMC_ID_VERIFY_ERROR;
}

/**
 * @brief  Perform a software reset on the MMC5983 device
 * @note   Sets the software reset bit (bit 7) in MMC_INTERNAL_CONTROL_1 register.
 *         Resets all internal control register shadow copies in the handler to zero.
 *         A delay of 15 ms is added to allow internal reset.
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @retval MMC5983_Error_TypeDef Status of the operation
 */
MMC5983_Error_TypeDef MMC5983_SW_Reset(MMC5983_HW_InitTypeDef *MMC5983_Handler){

	// Updates the expected content only for the Software Reset bit (to avoid writing in reserved bits):
	// Write the new content into INT__CTRL_1:
	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_1, MMC5983_SW_Reset_Mask)!=MMC_NO_ERROR)
	{
		return MMC_HAL_ERROR;
	}
	// Uses HAL_Delay to wait the power on time
	HAL_Delay(15);

	MMC5983_Handler->INT_CTRL_0_Reg = 0x00;
	MMC5983_Handler->INT_CTRL_1_Reg = 0x00;
	MMC5983_Handler->INT_CTRL_2_Reg = 0x00;
	MMC5983_Handler->INT_CTRL_3_Reg = 0x00;
	return (MMC_NO_ERROR);
}

/**
 * @brief  Set the bandwidth of the MMC5983 device
 * @note   Updates bits in MMC_INTERNAL_CONTROL_1 register to configure bandwidth.
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @param  MMC5983_BW_Rate Bandwidth setting bits to be OR'ed with INT_CTRL_1_Reg
 * @retval MMC5983_Error_TypeDef Status of the operation
 */
MMC5983_Error_TypeDef MMC5983_Set_Bandwidth(MMC5983_HW_InitTypeDef *MMC5983_Handler, uint8_t MMC5983_BW_Rate){

	MMC5983_Handler->INT_CTRL_1_Reg = MMC5983_Handler->INT_CTRL_1_Reg | MMC5983_BW_Rate;

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_1, MMC5983_Handler->INT_CTRL_1_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;

	return MMC_NO_ERROR;
}

/**
 * @brief  Set the operation rate and enable continuous measurement
 * @note   Updates MMC_INTERNAL_CONTROL_2 with the specified rate and enable bit
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @param  MMC5983_Set_Rate Operation rate bits to be set
 * @retval MMC5983_Error_TypeDef Status of the operation
 */
MMC5983_Error_TypeDef MMC5983_Set_Operation_Rate(MMC5983_HW_InitTypeDef *MMC5983_Handler, uint8_t MMC5983_Set_Rate){

	MMC5983_Handler->INT_CTRL_2_Reg = MMC5983_Handler->INT_CTRL_2_Reg | MMC5983_Set_Rate | MMC_SET_ENABLE;

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_2, MMC5983_Handler->INT_CTRL_2_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;
	else
		return MMC_NO_ERROR;
}

/**
 * @brief  Enable automatic set/reset function for each measurement
 * @note   Updates MMC_INTERNAL_CONTROL_0 with the auto set/reset enable bit
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @retval MMC5983_Error_TypeDef Status of the operation
 */
MMC5983_Error_TypeDef MMC5983_Enable_Auto_Set_Reset(MMC5983_HW_InitTypeDef *MMC5983_Handler){

	MMC5983_Handler->INT_CTRL_0_Reg = MMC5983_Handler->INT_CTRL_0_Reg | MMC_AUTO_SET_RESET_ENABLE;

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_0, MMC5983_Handler->INT_CTRL_0_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;
	else
		return MMC_NO_ERROR;
}

/**
 * @brief  Set the MMC5983 output data rate
 * @note   Uses MMC5983_SingleRegister_Write to write the specified
 *         value into MMC_INTERNAL_CONTROL_2 register
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @param  MMC5983_Rate 8-bit value of the Data Rate (see datasheet)
 * @retval MMC5983_Error_TypeDef
 */
MMC5983_Error_TypeDef MMC5983_Set_Output_DataRate(MMC5983_HW_InitTypeDef *MMC5983_Handler, uint8_t MMC5983_Rate){

	MMC5983_Handler->INT_CTRL_2_Reg = MMC5983_Handler->INT_CTRL_2_Reg | MMC5983_Rate;

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_2, MMC5983_Handler->INT_CTRL_2_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;
	else
		return MMC_NO_ERROR;
}

/**
 * @brief  Enable continuous measurement mode
 * @note   Sets MEAS_MMODE bit in MMC_INTERNAL_CONTROL_2 to enable continuous measurements
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @retval MMC5983_Error_TypeDef Status of the operation
 */
MMC5983_Error_TypeDef MMC5983_Set_Continuous_Measurement(MMC5983_HW_InitTypeDef *MMC5983_Handler){

	MMC5983_Handler->INT_CTRL_2_Reg = MMC5983_Handler->INT_CTRL_2_Reg | MMC_MEASUREMODE;

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_2, MMC5983_Handler->INT_CTRL_2_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;
	else
		return MMC_NO_ERROR;
}

/**
 * @brief  Enable interrupt on data ready for MMC5983
 * @note   Writes MMC_INT_DATA_READY to MMC_INTERNAL_CONTROL_0 register and clears interrupt by reading data
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @param  MMC5983_Data Pointer to the MMC5983_Data_TypeDef structure
 * @retval MMC5983_Error_TypeDef
 */
MMC5983_Error_TypeDef MMC5983_Set_Interrupt_Measurement(MMC5983_HW_InitTypeDef *MMC5983_Handler, MMC5983_Data_TypeDef *MMC5983_Data){

	MMC5983_Handler->INT_CTRL_0_Reg = MMC5983_Handler->INT_CTRL_0_Reg | MMC_INT_DATA_READY;

	// Route Data Ready to INT pin
	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_0, MMC5983_Handler->INT_CTRL_0_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;

	// Read data to clear interrupt
	MMC5983_Data_Read(MMC5983_Handler, MMC5983_Data);
	return MMC_NO_ERROR;
}

/**
 * @brief  Read X, Y, Z magnetometer data using sequential SPI access
 * @note   Enables multi-byte SPI access and reads 6 bytes from data registers
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @param  MMC5983_Data Pointer to the MMC5983_Data_TypeDef structure to store raw data
 * @retval MMC5983_Error_TypeDef Status of the operation
 */
MMC5983_Error_TypeDef MMC5983_Data_Read(MMC5983_HW_InitTypeDef *MMC5983_Handler, MMC5983_Data_TypeDef *MMC5983_Data){
	uint8_t SPITransmitData_MMC;
	uint8_t MMC_Data_Reply[7];

	SPITransmitData_MMC = (MMC_Read_Command | MMC_DATA_ADDRESS_INIT);


	enableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
	if(HAL_SPI_Transmit(MMC5983_Handler->SPIhandler, &SPITransmitData_MMC, 1, MMC5983_Handler->SPI_Timeout) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	if(HAL_SPI_Receive(MMC5983_Handler->SPIhandler, MMC_Data_Reply, 6, MMC5983_Handler->SPI_Timeout) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);


	MMC5983_Data->axes.DX =  (int16_t)((MMC_Data_Reply[0]<<8) |  MMC_Data_Reply[1]);
	MMC5983_Data->axes.DY =  (int16_t)((MMC_Data_Reply[2]<<8) |  MMC_Data_Reply[3]);
	MMC5983_Data->axes.DZ =  (int16_t)((MMC_Data_Reply[4]<<8) |  MMC_Data_Reply[5]);

	return MMC_NO_ERROR;
}

/**
 * @brief  Perform a single register write at the specified register
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @param  MMC5983_Register_Addr 8-bit register address to write to
 * @param  MMC5983_Write_Data 8-bit data to write
 * @retval MMC5983_Error_TypeDef
 */
MMC5983_Error_TypeDef MMC5983_SingleRegister_Write(MMC5983_HW_InitTypeDef *MMC5983_Handler, uint8_t MMC5983_Register_Addr, uint8_t MMC5983_Write_Data)
{
	uint8_t SPITransmitData_MMC;
	SPITransmitData_MMC = (MMC_Write_command | MMC5983_Register_Addr);

	enableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
	if(HAL_SPI_Transmit(MMC5983_Handler->SPIhandler, &SPITransmitData_MMC, 1, MMC5983_Handler->SPI_Timeout) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	if(HAL_SPI_Transmit(MMC5983_Handler->SPIhandler, &MMC5983_Write_Data, 1, MMC5983_Handler->SPI_Timeout) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);

	return MMC_NO_ERROR;
}

/**
 * @brief  Perform a single register read at the specified register
 * @param  MMC5983_Handler Pointer to the MMC5983_HW_InitTypeDef structure
 * @param  MMC5983_Register_Addr 8-bit register address to read from
 * @param  pData Pointer to the variable to store the read data
 * @retval MMC5983_Error_TypeDef
 */
MMC5983_Error_TypeDef MMC5983_SingleRegister_Read(MMC5983_HW_InitTypeDef *MMC5983_Handler, uint8_t MMC5983_Register_Addr, uint8_t *pData)
{
	uint8_t SPITransmitData_MMC;
	SPITransmitData_MMC = (MMC_Read_Command | MMC5983_Register_Addr);

	enableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
	if(HAL_SPI_Transmit(MMC5983_Handler->SPIhandler, &SPITransmitData_MMC, 1, MMC5983_Handler->SPI_Timeout) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	if(HAL_SPI_Receive(MMC5983_Handler->SPIhandler, pData, 1, MMC5983_Handler->SPI_Timeout) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);

	return MMC_NO_ERROR;
}

/**
 * @brief  Enable the chip select pin for SPI communication
 * @param  CS_GPIOport GPIO port of the chip select pin
 * @param  CS_GPIOpin GPIO pin number of the chip select
 * @retval None
 */
void enableCS_MMC5983(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin){
  HAL_GPIO_WritePin(CS_GPIOport, CS_GPIOpin, GPIO_PIN_RESET);
}

/**
 * @brief  Disable the chip select pin for SPI communication
 * @param  CS_GPIOport GPIO port of the chip select pin
 * @param  CS_GPIOpin GPIO pin number of the chip select
 * @retval None
 */
void disableCS_MMC5983(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin){
  HAL_GPIO_WritePin(CS_GPIOport, CS_GPIOpin, GPIO_PIN_SET);
}
