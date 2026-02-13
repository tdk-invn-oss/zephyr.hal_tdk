/*
 * Copyright (c) 2025 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */

#include "tad214x.h"
#include "tad214x_regmap.h"
#include <string.h>


#define CONST_KEY  0x5555
#define CONF0_KEY  0x5555
#define CONF1_KEY  0xAAAA
#define CALIB0_KEY 0x5555
#define CALIB1_KEY 0xAAAA
#define CALIB2_KEY 0x3333
#define CALIB3_KEY 0xCCCC
#define CALIB4_KEY 0xF0F0

// Static variable
static uint16_t SavedCalibHeader = 0;
static uint16_t SavedPredictionRatio = 0;

void TAD214x_get_time_us(struct tad214x *s, uint32_t us)
{
	if (s->serif.sleep_us != NULL)
	s->serif.sleep_us(us);
}

int TAD214x_GetData(struct tad214x *s, uint16_t *TMRData, int16_t *TempData)
{
	int rc = INV_ERROR_SUCCESS;

	rc =  TAD214x_ReadReg(&s->serif, DATA_REG_REG, TMRData);
	rc |= TAD214x_ReadReg(&s->serif, TEMP_REG_REG, (uint16_t *) TempData);

	return rc;
}
int TAD214x_Init(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	rc = TAD214x_SoftReset(s);
	return rc;
}

int TAD214x_SoftReset(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t WriteData = SEQUENCER_CTRL_SOFT_RESET_BIT;
	rc = TAD214x_WriteReg(&s->serif, SEQUENCER_CTRL_REG, &WriteData);
	
	/* Wait 2 ms for soft reset to be effective */
	TAD214x_get_time_us(s, 2000);
	return rc;
}

int TAD214x_SetMode(struct tad214x *s, TAD214X_PowerMode_t mode)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t TMRData = 0;
	uint16_t WriteData = mode;
	if ((mode == TAD214X_MODE_LPM) || (mode == TAD214X_MODE_CONT)) // Read Data to clear Interrupt
		rc = TAD214x_ReadReg(&s->serif, DATA_REG_REG, &TMRData);
	rc = TAD214x_ReadModifyWriteReg(&s->serif, MODE_CTRL_REG, WriteData, MODE_CTRL_MODE_SEL_MASK);
	return rc;
}

int TAD214x_GetMode(struct tad214x *s, TAD214X_PowerMode_t *Cur_Mode)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t data;

	rc = TAD214x_ReadReg(&s->serif, MODE_STATUS_REG, &data);
	if (rc)
		return rc;
	*Cur_Mode = (TAD214X_PowerMode_t)(data & MODE_STATUS_MODE_STATUS_MASK);

	return rc;
}

int TAD214x_SetODR(struct tad214x *s, TAD214X_ODR_t Odr)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t WriteData = (uint16_t) Odr << MODE_CTRL_LPM_ODR_POS;
	rc = TAD214x_ReadModifyWriteReg(&s->serif, MODE_CTRL_REG, WriteData, MODE_CTRL_LPM_ODR_MASK);
	return rc;	
}

int TAD214x_GetODR(struct tad214x *s, TAD214X_ODR_t * Odr)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t ReadData;

	rc = TAD214x_ReadReg(&s->serif, MODE_CTRL_REG, &ReadData);
	*Odr = (TAD214X_ODR_t)((ReadData & MODE_CTRL_LPM_ODR_MASK) >> MODE_CTRL_LPM_ODR_POS);
	
	return rc;		
}

int TAD214x_SetAngleOffset(struct tad214x *s, uint16_t AngleOffset, uint8_t cw_ccw_sign)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t WriteData;
	if (cw_ccw_sign == 1)
		WriteData = (AngleOffset & CALIB_SETTINGS_OUT_ADJ_OFFSET_MASK) | CALIB_SETTINGS_OUT_ADJ_CW_CCW_SIGN_BIT;
	else
		WriteData = (AngleOffset & CALIB_SETTINGS_OUT_ADJ_OFFSET_MASK);	
	rc |= TAD214x_WriteReg(&s->serif, CALIB_SETTINGS_OUT_ADJ_REG, &WriteData);
	return rc;
}

int TAD214x_GetAngleOffset(struct tad214x *s, uint16_t * AngleOffset, uint8_t * cw_ccw_sign)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t ReadData;
	
	rc = TAD214x_ReadReg(&s->serif, CALIB_SETTINGS_OUT_ADJ_REG, &ReadData);
	*AngleOffset = ReadData & CALIB_SETTINGS_OUT_ADJ_OFFSET_MASK;
	if ((ReadData & CALIB_SETTINGS_OUT_ADJ_CW_CCW_SIGN_BIT) == CALIB_SETTINGS_OUT_ADJ_CW_CCW_SIGN_BIT)
		*cw_ccw_sign = 1;
	else
		*cw_ccw_sign = 0;
	return rc;
}

int TAD214x_SetObserveDebugMode(struct tad214x *s, tad214x_observe_data_sel_t tad214x_observe_data_sel)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t WriteData = tad214x_observe_data_sel << DSP_DEBUG_OBSERVE_DATA_SEL_POS;
	rc |= TAD214x_ReadModifyWriteReg(&s->serif, DSP_DEBUG_REG, WriteData, DSP_DEBUG_OBSERVE_DATA_SEL_MASK);
	return rc;
}

int TAD214x_GetObserveData(struct tad214x *s, int16_t ObservData[2])
{
	int rc = INV_ERROR_SUCCESS;
	rc = TAD214x_ReadReg(&s->serif, DSP_OBSERVE_0_REG, (uint16_t *) &ObservData[0]);
	rc = TAD214x_ReadReg(&s->serif, DSP_OBSERVE_1_REG, (uint16_t *) &ObservData[1]);
	return rc;
}

int TAD214x_WriteReg(struct tad214x_serif *s, uint8_t Adr, uint16_t *Data)
{
	int rc = INV_ERROR_SUCCESS;
	rc = tad214x_serif_write_reg(s, Adr, Data, 1);
	return rc;
}
int TAD214x_ReadReg(struct tad214x_serif *s, uint8_t Adr, uint16_t *Data)
{
	int rc = INV_ERROR_SUCCESS;
	rc = tad214x_serif_read_reg(s, Adr, Data, 1);
	return rc;
}
int TAD214x_ReadModifyWriteReg(struct tad214x_serif *s, uint8_t Adr, uint16_t Data, uint16_t Mask)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t ReadData = 0;
	uint16_t WriteData = 0;
	rc = TAD214x_ReadReg(s, Adr, &ReadData);
	WriteData = (Data & Mask) | (ReadData & ~Mask);
	rc |= TAD214x_WriteReg(s, Adr, &WriteData);
	return rc;
}

int TAD214x_EnableSPIInterface(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	// SPI mode is active by assertion of CSB low > 1ms on ABZ mode.
	if (s->serif.if_tad == IF_ENC)
	{
		rc = tad214x_serif_cs_pulse(&s->serif);
		return rc;
	}
	return -1;
}

int TAD214x_GetOTPStatus(struct tad214x *s, int8_t * CONST_Status, int8_t * CONF_Status, int8_t * CALIB_Status)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t ReadKey = 0;
	rc = TAD214x_ReadReg(&s->serif, CONST_SETTINGS_CONST_HEADER_REG, &ReadKey);
	if (ReadKey == CONST_KEY)
		*CONST_Status = 0;
	else
		*CONST_Status = -1;
	rc = TAD214x_ReadReg(&s->serif, CONF_SETTINGS_CONF_HEADER_REG, &ReadKey);
	if (ReadKey == CONF0_KEY)
		*CONF_Status = 0;
	else if (ReadKey == CONF1_KEY)
		*CONF_Status = 1;
	else
		*CONF_Status = -1;
	rc = TAD214x_ReadReg(&s->serif, CALIB_SETTINGS_CALIB_HEADER_REG, &ReadKey);
	switch (ReadKey) {
		case CALIB0_KEY:
			*CALIB_Status = 0;
			break;
		case CALIB1_KEY:
			*CALIB_Status = 1;
			break;
		case CALIB2_KEY:
			*CALIB_Status = 2;
			break;
		case CALIB3_KEY:
			*CALIB_Status = 3;
			break;
		case CALIB4_KEY:
			*CALIB_Status = 4;
			break;
		default:
			*CALIB_Status = -1;
	}
	return rc;
}

int TAD214x_WriteOTP(struct tad214x_serif *s, uint16_t Adr, uint16_t Data)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t OTPStatus = OTP_STATUS_BUSY_BIT;
	uint16_t DataWr;
	
	// 3 LSB Adr are defined in Word Sel Register 
	DataWr = ((Adr>>3) << OTP_ADDRESS_ADDR_POS) & OTP_ADDRESS_ADDR_MASK;
	rc = TAD214x_WriteReg(s, OTP_ADDRESS_REG, &DataWr);
	DataWr = (Adr >> OTP_WORD_SEL_ADDR_POS) & OTP_WORD_SEL_ADDR_MASK;
	rc |= TAD214x_WriteReg(s, OTP_WORD_SEL_REG,&DataWr);
	DataWr = 1;
	rc |= TAD214x_WriteReg(s, OTP_MR_REG, &DataWr);
	DataWr = Data;
	rc |= TAD214x_WriteReg(s, OTP_WWORD_REG, &DataWr);
	DataWr = OTP_PROG_WORD;
	rc |= TAD214x_WriteReg(s, OTP_CTRL_REG, &DataWr);
	while ((OTPStatus & OTP_STATUS_BUSY_BIT) == OTP_STATUS_BUSY_BIT)
	{
		rc |= TAD214x_ReadReg(s, OTP_STATUS_REG, &OTPStatus);
	}
	return rc;
}

int TAD214x_ReadOTP(struct tad214x_serif *s, uint16_t Adr, uint16_t * Data)
{
		int rc = INV_ERROR_SUCCESS;
		uint16_t OTPStatus = OTP_STATUS_BUSY_BIT;
		uint16_t DataWr;
		
		// 3 LSB Adr are defined in Word Sel Register
		DataWr = ((Adr>>3) << OTP_ADDRESS_ADDR_POS) & OTP_ADDRESS_ADDR_MASK;
		rc = TAD214x_WriteReg(s, OTP_ADDRESS_REG, &DataWr);
		DataWr = (Adr >> OTP_WORD_SEL_ADDR_POS) & OTP_WORD_SEL_ADDR_MASK;
		rc |= TAD214x_WriteReg(s, OTP_WORD_SEL_REG,&DataWr);
		DataWr = 0x1412;
		rc |= TAD214x_WriteReg(s, OTP_MR_REG, &DataWr);
		DataWr = OTP_READ_INT;
		rc |= TAD214x_WriteReg(s, OTP_CTRL_REG, &DataWr);
		while ((OTPStatus & OTP_STATUS_BUSY_BIT) == OTP_STATUS_BUSY_BIT)
		{
			rc |= TAD214x_ReadReg(s, OTP_STATUS_REG, &OTPStatus);
		}
		rc |= TAD214x_ReadReg(s, OTP_RWORD_REG, Data);
		
		return rc;
}

int TAD214x_Unlock(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t DataWr = UNLOCK_KEY;
	rc = TAD214x_WriteReg(&s->serif, GLOBAL_LOCK_REG, &DataWr);
	return rc;
}

int TAD214x_Lock(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t DataWr = 0x0000;
	rc = TAD214x_WriteReg(&s->serif, GLOBAL_LOCK_REG, &DataWr);
	return rc;
}

int TAD214x_ProgramModeEnable(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t DataWr = SEQUENCER_CTRL_PROGRAM_EN_BIT;
	rc = TAD214x_WriteReg(&s->serif, SEQUENCER_CTRL_REG, &DataWr);
	return rc;
}

int TAD214x_ProgramModeDisable(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t DataWr = 0x0000;
	rc = TAD214x_WriteReg(&s->serif, SEQUENCER_CTRL_REG, &DataWr);
	return rc;
}

int TAD214x_PredictionEnable(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	rc = TAD214x_ReadModifyWriteReg(&s->serif, CONF_SETTINGS_SYS_CONFIG_REG, SavedPredictionRatio, CONF_SETTINGS_SYS_CONFIG_PREDICT_NM_RATIO_MASK);
	return rc;
}

int TAD214x_PredictionDisable(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	// Save Previous Key to use when we re enable
	rc |= TAD214x_ReadReg(&s->serif, CALIB_SETTINGS_CALIB_HEADER_REG, &SavedPredictionRatio);
	uint16_t DataWr = 0x0000;
	rc = TAD214x_ReadModifyWriteReg(&s->serif, CONF_SETTINGS_SYS_CONFIG_REG, DataWr, CONF_SETTINGS_SYS_CONFIG_PREDICT_NM_RATIO_MASK);
	return rc;
}

int TAD214x_EnableDynamicCompensation(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	rc = TAD214x_ReadModifyWriteReg(&s->serif, CONF_SETTINGS_DSP_CONFIG_REG, 0x0000, CONF_SETTINGS_DSP_CONFIG_DISABLE_NORMALIZE_BIT);
	return rc;
}

int TAD214x_DisableDynamicCompensation(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	rc = TAD214x_ReadModifyWriteReg(&s->serif, CONF_SETTINGS_DSP_CONFIG_REG, CONF_SETTINGS_DSP_CONFIG_DISABLE_NORMALIZE_BIT, CONF_SETTINGS_DSP_CONFIG_DISABLE_NORMALIZE_BIT);
	return rc;
}

int TAD214x_EnableLUTCompensation(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	if (SavedCalibHeader != 0x0000)
		rc = TAD214x_WriteReg(&s->serif, CALIB_SETTINGS_CALIB_HEADER_REG, &SavedCalibHeader);
	return rc;
}

int TAD214x_DisableLUTCompensation(struct tad214x *s)
{
	int rc = INV_ERROR_SUCCESS;
	uint16_t Calib_Header = 0x0000;
	// Save Previous Key to use when we re enable
	rc |= TAD214x_ReadReg(&s->serif, CALIB_SETTINGS_CALIB_HEADER_REG, &Calib_Header);
	if (Calib_Header != 0x0000)
	{
		SavedCalibHeader = Calib_Header;
		uint16_t DataWr = 0x0000;
		rc = TAD214x_WriteReg(&s->serif, CALIB_SETTINGS_CALIB_HEADER_REG, &DataWr);
	}
	return rc;
}

int TAD214x_SetSinCosGain(struct tad214x *s, uint16_t SinCosGain[2])
{
	int rc = INV_ERROR_SUCCESS;
	rc =  TAD214x_WriteReg(&s->serif, CALIB_SETTINGS_SIN_LIN_GAIN_IC_REG, &SinCosGain[0]);
	rc |= TAD214x_WriteReg(&s->serif, CALIB_SETTINGS_COS_GAIN_IC_REG,     &SinCosGain[1]);
	return rc;
}

int TAD214x_GetSinCosGain(struct tad214x *s, uint16_t SinCosGain[2])
{
	int rc = INV_ERROR_SUCCESS;
	rc =  TAD214x_ReadReg(&s->serif, CALIB_SETTINGS_SIN_LIN_GAIN_IC_REG,  &SinCosGain[0]);
	rc |= TAD214x_ReadReg(&s->serif, CALIB_SETTINGS_COS_GAIN_IC_REG,      &SinCosGain[1]);
	return rc;
}

int TAD214x_SetSinCosOffset(struct tad214x *s, int16_t SinCosOffset[2])
{
	int rc = INV_ERROR_SUCCESS;
	rc =  TAD214x_WriteReg(&s->serif, CALIB_SETTINGS_SIN_LIN_OFFS_IC_REG,  (uint16_t *) &SinCosOffset[0]);
	rc |= TAD214x_WriteReg(&s->serif, CALIB_SETTINGS_COS_OFFS_IC_REG,      (uint16_t *) &SinCosOffset[1]);
	return rc;
}

int TAD214x_GetSinCosOffset(struct tad214x *s, int16_t SinCosOffset[2])
{
	int rc = INV_ERROR_SUCCESS;
	rc =  TAD214x_ReadReg(&s->serif, CALIB_SETTINGS_SIN_LIN_OFFS_IC_REG, (uint16_t *) &SinCosOffset[0]);
	rc |= TAD214x_ReadReg(&s->serif, CALIB_SETTINGS_COS_OFFS_IC_REG,     (uint16_t *) &SinCosOffset[1]);
	return rc;
}

int TAD214x_GetCurrentSinCosGain(struct tad214x *s, uint16_t SinCosGain[2])
{
	int rc = INV_ERROR_SUCCESS;
	rc =  TAD214x_ReadReg(&s->serif, SIN_GAIN_REG,  &SinCosGain[0]);
	rc |= TAD214x_ReadReg(&s->serif, COS_GAIN_REG,  &SinCosGain[1]);
	return rc;
}

int TAD214x_GetCurrentSinCosOffset(struct tad214x *s, int16_t SinCosOffset[2])
{
	int rc = INV_ERROR_SUCCESS;
	rc =  TAD214x_ReadReg(&s->serif, SIN_OFFSET_REG, (uint16_t *) &SinCosOffset[0]);
	rc |= TAD214x_ReadReg(&s->serif, COS_OFFSET_REG, (uint16_t *) &SinCosOffset[1]);
	return rc;
}