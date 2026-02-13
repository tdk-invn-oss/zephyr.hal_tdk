// Converter version 1.0.0.0
// Source: Register_map_TAD2144_rev04.xlsx , 2025-05-16 4:35:06 PM , MD5:FEE42BEA395626E2E2891A5212E075B3
#define CONST_SETTINGS_CONST_HEADER_REG               0x00 // CONST area header

#define CONST_SETTINGS_TRIM0_REG                      0x01 // Analog trimming
#define     CONST_SETTINGS_TRIM0_LPREF_IBIAS_MASK     0x000F // LP-reference bias current trimming (2's complement)
#define     CONST_SETTINGS_TRIM0_LPREF_IBIAS_POS      0
#define     CONST_SETTINGS_TRIM0_LPREF_VREF_MASK      0x01F0 // LP-reference reference voltage trimming (2's complement)
#define     CONST_SETTINGS_TRIM0_LPREF_VREF_POS       4

#define CONST_SETTINGS_TRIM1_REG                      0x02 // Analog trimming
#define     CONST_SETTINGS_TRIM1_BG_PTAT_MASK         0x000F // Bandgap PTAT current trimming (unsigned)
#define     CONST_SETTINGS_TRIM1_BG_PTAT_POS          0
#define     CONST_SETTINGS_TRIM1_BG_VBG_MASK          0x01F0 // Bandgap offset (2's complement)
#define     CONST_SETTINGS_TRIM1_BG_VBG_POS           4

#define CONST_SETTINGS_TRIM2_REG                      0x03 // Analog trimming
#define     CONST_SETTINGS_TRIM2_HFO_MASK             0x007F // HFO trimming (unsigned)
#define     CONST_SETTINGS_TRIM2_HFO_POS              0
#define     CONST_SETTINGS_TRIM2_LFO_MASK             0x0780 // LFO trimming (2's complement)
#define     CONST_SETTINGS_TRIM2_LFO_POS              7
#define     CONST_SETTINGS_TRIM2_LDOTMR_MASK          0x3800 // LDOTMR output voltage trimming (unsigned)
#define     CONST_SETTINGS_TRIM2_LDOTMR_POS           11

#define CONST_SETTINGS_TRIM3_REG                      0x04 // Analog/Temperature sensor trimming
#define     CONST_SETTINGS_TRIM3_DVBE_C_MASK          0x000F // Temperature sensor delta VBE coarse trim code (unsigned)
#define     CONST_SETTINGS_TRIM3_DVBE_C_POS           0
#define     CONST_SETTINGS_TRIM3_DVBE_F_MASK          0x01F0 // Temperature sensor delta VBE fine trim code (unsigned)
#define     CONST_SETTINGS_TRIM3_DVBE_F_POS           4
#define     CONST_SETTINGS_TRIM3_LDOA_MASK            0x0600 // LDOA output voltage trimming (2's complement)
#define     CONST_SETTINGS_TRIM3_LDOA_POS             9

#define CONST_SETTINGS_TRIM4_REG                      0x05 // Temperature sensor trimming
#define     CONST_SETTINGS_TRIM4_VBE_C_MASK           0x000F // Temperature sensor VBE coarse trim code (unsigned)
#define     CONST_SETTINGS_TRIM4_VBE_C_POS            0
#define     CONST_SETTINGS_TRIM4_VBE_F_MASK           0x01F0 // Temperature sensor VBE fine trim code (unsigned)
#define     CONST_SETTINGS_TRIM4_VBE_F_POS            4
#define     CONST_SETTINGS_TRIM4_SPARE_MASK           0x0C00
#define     CONST_SETTINGS_TRIM4_SPARE_POS            10
#define     CONST_SETTINGS_TRIM4_SPARE_ISO_MASK       0x3000
#define     CONST_SETTINGS_TRIM4_SPARE_ISO_POS        12
#define     CONST_SETTINGS_TRIM4_SPARELS_MASK         0xC000
#define     CONST_SETTINGS_TRIM4_SPARELS_POS          14

#define CONST_SETTINGS_TEMP_OFFSET_REG                0x06 // Temperature sensor calibration
#define     CONST_SETTINGS_TEMP_OFFSET_TEMP_OFFSET_MASK 0x01FF // Temperature sensor offset calibration
#define     CONST_SETTINGS_TEMP_OFFSET_TEMP_OFFSET_POS 0
#define     CONST_SETTINGS_TEMP_OFFSET_TEMP_DECIM_MASK 0xFE00 // Temperature sensor decimation factor calibration
#define     CONST_SETTINGS_TEMP_OFFSET_TEMP_DECIM_POS 9

#define CONST_SETTINGS_TEMP_SLOPE_REG                 0x07 // Temperature sensor calibration
#define     CONST_SETTINGS_TEMP_SLOPE_TEMP_SLOPE_MASK 0x00FF // Temperature sensor slope calibration
#define     CONST_SETTINGS_TEMP_SLOPE_TEMP_SLOPE_POS  0

#define CONST_SETTINGS_AFE_CONFIGURATION_REG          0x08 // AFE configuration
#define     CONST_SETTINGS_AFE_CONFIGURATION_AFE_IA_GAIN_MASK 0x0007 // AFE IA gain selection
#define     CONST_SETTINGS_AFE_CONFIGURATION_AFE_IA_GAIN_POS 0
#define     CONST_SETTINGS_AFE_CONFIGURATION_SWAP_SIN_COS_BIT 0x0008 // Swap SIN and COS channels
#define     CONST_SETTINGS_AFE_CONFIGURATION_INV_SIN_BIT 0x0010 // Invert SIN channel
#define     CONST_SETTINGS_AFE_CONFIGURATION_INV_COS_BIT 0x0020 // Invert COS channel
#define     CONST_SETTINGS_AFE_CONFIGURATION_IA_OFFSET_MASK 0x0FC0 // IA offset trimming. MSB selects sign, offset code is unsigned.
#define     CONST_SETTINGS_AFE_CONFIGURATION_IA_OFFSET_POS 6
#define     CONST_SETTINGS_AFE_CONFIGURATION_CLK_SEL_BIT 0x1000 // Invert the clock sampling adc data before decimation

#define CONST_SETTINGS_CONST_CRC_REG                  0x09 // Constant area CRC

#define CONF_SETTINGS_CONF_HEADER_REG                 0x10 // CONF area header

#define CONF_SETTINGS_DSP_CONFIG_REG                  0x11 // DSP configuration
#define     CONF_SETTINGS_DSP_CONFIG_NORM_GAIN_FB_MASK 0x000F // Feedback factor for the normalization update (Unsigned 4 bit, 0 to 2-2^(-3)). Smaller values mean slower updates and higher values mean faster updates and higher suscetibility for noise. Any value >1.0 makes the system unstable.
#define     CONF_SETTINGS_DSP_CONFIG_NORM_GAIN_FB_POS 0
#define     CONF_SETTINGS_DSP_CONFIG_NORM_OFFS_FB_MASK 0x00F0 // Feedback factor for the normalization update (Unsigned 4 bit, 0 to 2-2^(-3)). Smaller values mean slower updates and higher values mean faster updates and higher suscetibility for noise. Any value >1.0 makes the system unstable.
#define     CONF_SETTINGS_DSP_CONFIG_NORM_OFFS_FB_POS 4
#define     CONF_SETTINGS_DSP_CONFIG_DISABLE_NORMALIZE_BIT 0x0100 // If set, force sin-cos normalization to static programmed values (gain_ic, offset_ic)
#define     CONF_SETTINGS_DSP_CONFIG_LIN_MODE_BIT     0x0200 // 0: angular mode
#define     CONF_SETTINGS_DSP_CONFIG_STARTUP_TIMER_MASK 0xC000
#define     CONF_SETTINGS_DSP_CONFIG_STARTUP_TIMER_POS 14

#define CONF_SETTINGS_SYS_CONFIG_REG                  0x12 // System config
#define     CONF_SETTINGS_SYS_CONFIG_PREDICT_NM_RATIO_MASK 0x03FF // Prediction N/M ratio (N is the latency in samples, M is fixed to 12). Format is 0 integer bits, 10 fractional, assumed as unsigned number. Setting the ratio to 0 disables the predictor.
#define     CONF_SETTINGS_SYS_CONFIG_PREDICT_NM_RATIO_POS 0
#define     CONF_SETTINGS_SYS_CONFIG_DIS_STARTUP_INT_BIT 0x0800 // Disable Startup completion interrupt
#define     CONF_SETTINGS_SYS_CONFIG_I2C_ADDR_EN_BIT  0x1000 // I2C slave address selection method.
                                                                        // 0: I2C address based on SA0 and SA1
                                                                        // 1: I2C address based on I2C_ADDR Register

#define     CONF_SETTINGS_SYS_CONFIG_I2C_ADDR_MASK    0x6000 // I2C_ADDR:
                                                                        // 0: 0x17
                                                                        // 1: 0x23
                                                                        // 2: 0x56
                                                                        // 3: 0x65

#define     CONF_SETTINGS_SYS_CONFIG_I2C_ADDR_POS     13
#define     CONF_SETTINGS_SYS_CONFIG_SLEEP_EN_BIT     0x8000 // Enable SLEEP state

#define CONF_SETTINGS_IF_CONFIG0_REG                  0x13 // Interface Settings
#define     CONF_SETTINGS_IF_CONFIG0_SPI_MODE_BIT     0x0001 // SPI Mode:
                                                                        // 0: SPI4
                                                                        // 1: SPI3

#define     CONF_SETTINGS_IF_CONFIG0_SPI_I2C_SEL_BIT  0x0002 // Interface Selection
                                                                        // 0: SPI Selected
                                                                        // 1: I2C Selected

#define     CONF_SETTINGS_IF_CONFIG0_ABZ_EN_BIT       0x0004 // Enable ABZ interface
#define     CONF_SETTINGS_IF_CONFIG0_ABZ_STEP_MODE_BIT 0x0008 // ABZ: Step direction mode
#define     CONF_SETTINGS_IF_CONFIG0_ABZ_STARTUP_BIT  0x0010 // ABZ: Increment from zero to current angle at activation
#define     CONF_SETTINGS_IF_CONFIG0_ABZ_Z_PULSE_WIDTH_MASK 0x03E0 // ABZ: Duration of Z pulse in 2us steps. Code 0 = 2us, code 31=64us
#define     CONF_SETTINGS_IF_CONFIG0_ABZ_Z_PULSE_WIDTH_POS 5
#define     CONF_SETTINGS_IF_CONFIG0_ABZ_AB_MIN_PULSE_WIDTH_MASK 0x3C00 // ABZ: Minimum Pulse width settings for the A and B outputs
                                                                        // 0: 0.125?s
                                                                        // 1: 0.25?s
                                                                        // 2: 0.50?s
                                                                        // 3: 0.75?s
                                                                        // 4: 1.00?s
                                                                        // 5: 1.25?s
                                                                        // 6: 1.50?s
                                                                        // 7: 1.75?s
                                                                        // 8: 2.00?S
                                                                        // 9: 2.25?S
                                                                        // 10: 2.50?S
                                                                        // 11: 2.75?S
                                                                        // 12: 3.00?S
                                                                        // 13: 3.25?s
                                                                        // 14: 3.50?S
                                                                        // 15: 3.75?S

#define     CONF_SETTINGS_IF_CONFIG0_ABZ_AB_MIN_PULSE_WIDTH_POS 10
#define     CONF_SETTINGS_IF_CONFIG0_ABZ_Z_PULSE_CONFIG_MASK 0xC000 // ABZ: Z pulse mode configuration
                                                                        // 0: 1LSB X4-Decoding
                                                                        // 1: 3LSB X4-Decoding
                                                                        // 2: Fixed Duration (Together with abz_z_pulse_width setting
                                                                        // 3: 180DEG Mode

#define     CONF_SETTINGS_IF_CONFIG0_ABZ_Z_PULSE_CONFIG_POS 14

#define CONF_SETTINGS_IF_CONFIG1_REG                  0x14 // Interface Settings
#define     CONF_SETTINGS_IF_CONFIG1_ABZ_PPR_MASK     0x0FFF // ABZ: pulses per rotation (PPR = register value+1)
#define     CONF_SETTINGS_IF_CONFIG1_ABZ_PPR_POS      0
#define     CONF_SETTINGS_IF_CONFIG1_CSB_LOW_DETECT_ENABLE_BIT 0x1000 // Enable the pulldown detection of the CSB line to exit ABZ mode
#define     CONF_SETTINGS_IF_CONFIG1_SPI_DSP_SAMPLE_MARGIN_MASK 0xE000 // Sets the nr. of sclk periods before the end of the stream command, to sample the dsp_angle_value
                                                                        // 0: 3 sclk periods
                                                                        // 1: 1 sclk periods
                                                                        // 2: 2 sclk periods
                                                                        // 3: 3 sclk periods
                                                                        // 4: 4 sclk periods
                                                                        // 5: 5 sclk periods
                                                                        // 6: resv
                                                                        // 7: resv

#define     CONF_SETTINGS_IF_CONFIG1_SPI_DSP_SAMPLE_MARGIN_POS 13

#define CONF_SETTINGS_IF_CONFIG2_REG                  0x15 // Interface Settings
#define     CONF_SETTINGS_IF_CONFIG2_SPI_I2C_DRV_MASK 0x0003 // SPI and I2C IO Drive Strength:
                                                                        // 0: 1mA
                                                                        // 1: 2mA
                                                                        // 2: 4mA
                                                                        // 3: 8mA

#define     CONF_SETTINGS_IF_CONFIG2_SPI_I2C_DRV_POS  0
#define     CONF_SETTINGS_IF_CONFIG2_INT_DRV_MASK     0x000C // Int IO drive Strength:
                                                                        // 0: 1mA
                                                                        // 1: 2mA
                                                                        // 2: 4mA
                                                                        // 3: 8mA

#define     CONF_SETTINGS_IF_CONFIG2_INT_DRV_POS      2
#define     CONF_SETTINGS_IF_CONFIG2_ABZ_DRV_MASK     0x0030 // abz IO drive Strength:
                                                                        // 0: 1mA
                                                                        // 1: 2mA
                                                                        // 2: 4mA
                                                                        // 3: 8mA

#define     CONF_SETTINGS_IF_CONFIG2_ABZ_DRV_POS      4
#define     CONF_SETTINGS_IF_CONFIG2_INT_PU_DIS_BIT   0x0040 // Disable weak pullup on INT pad
#define     CONF_SETTINGS_IF_CONFIG2_SCLK_PU_DIS_BIT  0x0080 // Disable weak pullup on SCLK pad
#define     CONF_SETTINGS_IF_CONFIG2_CSB_PU_DIS_BIT   0x0100 // Disable weak pullup on CSB pad
#define     CONF_SETTINGS_IF_CONFIG2_MOSI_PU_EN_BIT   0x0200 // Enable weak pullup on MOSI pad
#define     CONF_SETTINGS_IF_CONFIG2_HYST_ENABLE_BIT  0x0400 // Activate Hysteresis on ABZ interface
#define     CONF_SETTINGS_IF_CONFIG2_HYST_HYST_MASK   0x1800 // Hysteresis:
                                                                        // 0: 4 LSB Hysteresis
                                                                        // 1: 8 LSB Hysteresis
                                                                        // 2: 16 LSB Hysteresis
                                                                        // 3: 32 LSB Hysteresis

#define     CONF_SETTINGS_IF_CONFIG2_HYST_HYST_POS    11
#define     CONF_SETTINGS_IF_CONFIG2_HYST_OFFSET_CONFIG_MASK 0x6000 // Hyst: Enable/Disable the catch-up (Disabling introduces an offset in CW or CCW direction)
                                                                        // 0: Hysteresis Catch-up Enabled
                                                                        // 1: Hysteresis Catch-up Disabled, Offset in CW Direction
                                                                        // 2: Hysteresis Catch-up Disabled, Offset in CCW Direction
                                                                        // 3: Hysteresis Catch-up Enabled

#define     CONF_SETTINGS_IF_CONFIG2_HYST_OFFSET_CONFIG_POS 13

#define CONF_SETTINGS_CONF_CRC_REG                    0x16 // CONF area CRC

#define CALIB_SETTINGS_CALIB_HEADER_REG               0x20 // CALIB area header

#define CALIB_SETTINGS_LUT_OUT0_REG                   0x21 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT1_REG                   0x22 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT2_REG                   0x23 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT3_REG                   0x24 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT4_REG                   0x25 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT5_REG                   0x26 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT6_REG                   0x27 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT7_REG                   0x28 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT8_REG                   0x29 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT9_REG                   0x2A // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT10_REG                  0x2B // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT11_REG                  0x2C // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT12_REG                  0x2D // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT13_REG                  0x2E // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT14_REG                  0x2F // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT15_REG                  0x30 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT16_REG                  0x31 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT17_REG                  0x32 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT18_REG                  0x33 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT19_REG                  0x34 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT20_REG                  0x35 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT21_REG                  0x36 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT22_REG                  0x37 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT23_REG                  0x38 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT24_REG                  0x39 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT25_REG                  0x3A // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT26_REG                  0x3B // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT27_REG                  0x3C // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT28_REG                  0x3D // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT29_REG                  0x3E // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT30_REG                  0x3F // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_LUT_OUT31_REG                  0x40 // LUT output point (32 input points are mapped to 32 output points)

#define CALIB_SETTINGS_SIN_LIN_GAIN_IC_REG            0x41 // Gain initial condition for the normalize block

#define CALIB_SETTINGS_COS_GAIN_IC_REG                0x42 // Gain initial condition for the normalize block

#define CALIB_SETTINGS_SIN_LIN_OFFS_IC_REG            0x43 // Offset initial condition for the normalize block

#define CALIB_SETTINGS_COS_OFFS_IC_REG                0x44 // Offset initial condition for the normalize block

#define CALIB_SETTINGS_SIN_GAIN_TC_REG                0x45 // Sin/Lin channel gain temperature slope
#define     CALIB_SETTINGS_SIN_GAIN_TC_GAIN_TC_MASK   0x0FFF // Sin/Lin channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     CALIB_SETTINGS_SIN_GAIN_TC_GAIN_TC_POS    0
#define     CALIB_SETTINGS_SIN_GAIN_TC_TREF_3_0_MASK  0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     CALIB_SETTINGS_SIN_GAIN_TC_TREF_3_0_POS   12

#define CALIB_SETTINGS_SIN_OFFS_TC_REG                0x46 // Sin/Lin channel offset temperature slope
#define     CALIB_SETTINGS_SIN_OFFS_TC_OFFS_TC_MASK   0x0FFF // Sin/Lin channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     CALIB_SETTINGS_SIN_OFFS_TC_OFFS_TC_POS    0
#define     CALIB_SETTINGS_SIN_OFFS_TC_TREF_7_4_MASK  0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [7:4
#define     CALIB_SETTINGS_SIN_OFFS_TC_TREF_7_4_POS   12

#define CALIB_SETTINGS_COS_GAIN_TC_REG                0x47 // Cos channel gain temperature slope
#define     CALIB_SETTINGS_COS_GAIN_TC_GAIN_TC_MASK   0x0FFF // Cos channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     CALIB_SETTINGS_COS_GAIN_TC_GAIN_TC_POS    0
#define     CALIB_SETTINGS_COS_GAIN_TC_TREF_11_8_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     CALIB_SETTINGS_COS_GAIN_TC_TREF_11_8_POS  12

#define CALIB_SETTINGS_COS_OFFS_TC_REG                0x48 // Cos channel offset temperature slope
#define     CALIB_SETTINGS_COS_OFFS_TC_OFFS_TC_MASK   0x0FFF // Cos channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     CALIB_SETTINGS_COS_OFFS_TC_OFFS_TC_POS    0
#define     CALIB_SETTINGS_COS_OFFS_TC_TREF_12_BIT    0x1000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bit [12

#define CALIB_SETTINGS_OUT_ADJ_REG                    0x49 // Output adjustment settings
#define     CALIB_SETTINGS_OUT_ADJ_OFFSET_MASK        0x7FFF // Output angle offset (LSB=360*2^-15 deg)
#define     CALIB_SETTINGS_OUT_ADJ_OFFSET_POS         0
#define     CALIB_SETTINGS_OUT_ADJ_CW_CCW_SIGN_BIT    0x8000 // CW/CCW output adjustment:
                                                                        // 0: CCW No output adjustment
                                                                        // 1: CW (360 - measure angle)


#define CALIB_SETTINGS_CALIB_CRC_REG                  0x4A // CALIB area CRC

#define OTP_CTRL_REG                                  0x50 // OTP control
#define     OTP_CTRL_CMD_MASK                         0x0007 // OTP command:
                                                                        // 0: NOP
                                                                        // 1: READ_INT
                                                                        // 2: READ_EXT
                                                                        // 3: PROG_BITCELL
                                                                        // 4: PROG_WORD
                                                                        // 5: SOAK_BITCELL
                                                                        // 6: READ_EXT_MAX
                                                                        // 7: SOAK_WORD

#define     OTP_CTRL_CMD_POS                          0
#define     OTP_CTRL_MAX_SOAKING_CYCLES_MASK          0x0078 // OTP maximum number of soaking cycles to be used during the OTP_SOAK_WORD command
#define     OTP_CTRL_MAX_SOAKING_CYCLES_POS           3
#define     OTP_CTRL_MIN_SOAKING_CYCLES_BIT           0x0080 // OTP minimum number of soaking cycles to be used during the OTP_SOAK_WORD command

#define OTP_ADDRESS_REG                               0x51 // OTP address and bitsel
#define     OTP_ADDRESS_BIT_CELL_SEL_MASK             0x0003 // OTP Cell Address
#define     OTP_ADDRESS_BIT_CELL_SEL_POS              0
#define     OTP_ADDRESS_ADDR_MASK                     0x007C // 5 MSB of OTP Word address  (Corresponds to (otp_mir register address)/8)
#define     OTP_ADDRESS_ADDR_POS                      2

#define OTP_WORD_SEL_REG                              0x52 // OTP word selection
#define     OTP_WORD_SEL_ADDR_MASK                    0x0007 // 3 LSB of word address
#define     OTP_WORD_SEL_ADDR_POS                     0

#define OTP_MR_REG                                    0x53 // OTP MR bits to be programmed

#define OTP_WWORD_REG                                 0x54 // OTP word to be programmed

#define OTP_RWORD_REG                                 0x55 // OTP word read

#define OTP_STATUS_REG                                0x56 // OTP controller status
#define     OTP_STATUS_BUSY_BIT                       0x0001 // OTP controller BUSY flag
#define     OTP_STATUS_SOAK_OK_BIT                    0x0002 // Flag indicating if the soak operation was successful
#define     OTP_STATUS_USED_SOAK_CYCLES_MASK          0x003C // The number of soaked cycles used during the OTP_SOAK_WORD command
#define     OTP_STATUS_USED_SOAK_CYCLES_POS           2

#define OTP_OVERRIDE_REG                              0x57 // OTP control
#define     OTP_OVERRIDE_ENABLE_MASK                  0x0007 // Enable the override signals for the OTP access by writing "110" to this field
#define     OTP_OVERRIDE_ENABLE_POS                   0
#define     OTP_OVERRIDE_SEL_BIT                      0x0010 // Override for the OTP SEL signal
#define     OTP_OVERRIDE_WE_BIT                       0x0020 // Override for the OTP WE signal
#define     OTP_OVERRIDE_CK_BIT                       0x0040 // Override for the OTP CK signal
#define     OTP_OVERRIDE_PU_BIT                       0x0080 // Override for the OTP PU signal

#define OTP_DEBUG_DONE_REG                            0x58 // OTP DONE output signal for debug, only valid when otp_override.Enable is valid
#define     OTP_DEBUG_DONE_OTP_DEBUG_BIT              0x0001 // OTP debug done signal

#define DSP_DEBUG_REG                                 0x60 // DSP Debug register



#define     DSP_DEBUG_MAC_EN_BIT                      0x0002 // Enable MAC unit. When disabled, the MAC unit does not write data into DATA_REG
#define     DSP_DEBUG_SKIP_DEL_EQU_BIT                0x0004 // Disable the delay equalizer of the datapath
#define     DSP_DEBUG_MAC_DEBUG_START_EN_BIT          0x0008 // Enable debug mac start calculation overwrite. When enabled, the MAC unit is triggered only when mac_debug_start_pulse field is written with "1"
#define     DSP_DEBUG_MAC_DEBUG_START_PULSE_BIT       0x0010 // MAC trigger to start processing an ADC sample when mac_debug_start_en is "1". This is a pulse field (WO)
#define     DSP_DEBUG_DEBUG_ANGLE_GEN_EN_BIT          0x0020 // Enable debug angle generator overwrite
#define     DSP_DEBUG_ANGLE_GEN_SPEED_MASK            0x00C0 // Angle Generator Speed
                                                                        // 0: Slowest Rotation 1 incr per dsp cycle (30.9 RPM)
                                                                        // 1: Slow Rotation 32 incr per dsp cycle (988.9 RPM)
                                                                        // 2: Fast Rotation 485 incr per dsp cycle (15k RPM)
                                                                        // 3: Fastest Rotation 647 incr per dsp cycle (20k RPM)

#define     DSP_DEBUG_ANGLE_GEN_SPEED_POS             6
#define     DSP_DEBUG_OBSERVE_DATA_SEL_MASK           0x0700 // Debug Observe Selection
                                                                        // 0: DISABLED
                                                                        // 1: SIN_COS_RAW
                                                                        // 2: SIN_COS_COMP
                                                                        // 3: LUT_IN
                                                                        // 4: LUT_OUT
                                                                        // 5: PRED_OUT
                                                                        // 6: OUT_ADJ_OUT
                                                                        // 7: RSVD

#define     DSP_DEBUG_OBSERVE_DATA_SEL_POS            8

#define SIN_GAIN_REG                                  0x61 // Current gain used for the normalize block. Notice that this is a truncated version of the internal gain to match gain_ic format.

#define COS_GAIN_REG                                  0x62 // Current gain used for the normalize block. Notice that this is a truncated version of the internal gain to match gain_ic format.

#define SIN_OFFSET_REG                                0x63 // Current offset for the normalize block

#define COS_OFFSET_REG                                0x64 // Current offset for the normalize block

#define MAC_DEBUG_PC_UPDATE_REG                       0x65 // MAC debug register update control
#define     MAC_DEBUG_PC_UPDATE_MAC_DEBUG_PC_UPDATE_MASK 0x003F // Program Counter at which the mac_debug_reg value is updated
#define     MAC_DEBUG_PC_UPDATE_MAC_DEBUG_PC_UPDATE_POS 0

#define MAC_DEBUG_REG_MSB_REG                         0x66 // MAC unit value registered on program counter matching (MSB)

#define MAC_DEBUG_REG_LSB_REG                         0x67 // MAC unit value registered on program counter matching (LSB)
#define     MAC_DEBUG_REG_LSB_MAC_DEBUG_REG_LSB_MASK  0x001F // 5 LSBs of Fi(1;21;16) (signal[4:0])
#define     MAC_DEBUG_REG_LSB_MAC_DEBUG_REG_LSB_POS   0

#define MODE_CTRL_REG                                 0x80 // Mode control
#define     MODE_CTRL_MODE_SEL_MASK                   0x0003 // Mode Selection:
                                                                        // 0: Standby or sleep
                                                                        // 1: Continuous
                                                                        // 2: LPM
                                                                        // 3: Single Shot (Auto clear)

#define     MODE_CTRL_MODE_SEL_POS                    0
#define     MODE_CTRL_LPM_ODR_MASK                    0x000C // Output Data rate in LPM
                                                                        // 0: ODR 10Hz
                                                                        // 1: ODR 100Hz
                                                                        // 2: ODR 300Hz
                                                                        // 3: RSVD

#define     MODE_CTRL_LPM_ODR_POS                     2
#define     MODE_CTRL_TEMP_SEL_BIT                    0x0010 // Temperature readout selection:
                                                                        // 0: Temp Filtered
                                                                        // 1: Temp Raw

#define     MODE_CTRL_POR_DIS_SLEEP_BIT               0x8000 // Disable POR circuit in SLEEP stateDisable POR circuit in SLEEP stateDisable POR circuit in SLEEP state

#define DATA_REG_REG                                  0x81 // Angle or Magnetic Field read-out value

#define TEMP_REG_REG                                  0x82 // Temperature read-out value

#define MODE_STATUS_REG                               0x83 // Mode status
#define     MODE_STATUS_MODE_STATUS_MASK              0x0003 // Mode Status:
                                                                        // 0: Standby or sleep
                                                                        // 1: Continuous
                                                                        // 2: LPM
                                                                        // 3: Single Shot (Auto clear)

#define     MODE_STATUS_MODE_STATUS_POS               0

#define DSP_OBSERVE_0_REG                             0x90 // DSP observe register. In CONTINUOUS mode, reading dsp_observe_0 latches the value in dsp_observe_1 to ensure data consistency. In LPM/SINGLE-SHOT, these registers are only expected to be read when the DSP is not running.

#define DSP_OBSERVE_1_REG                             0x91 // DSP observe register. In CONTINUOUS mode, reading dsp_observe_0 latches the value in dsp_observe_1 to ensure data consistency. In LPM/SINGLE-SHOT, these registers are only expected to be read when the DSP is not running.

#define SEQUENCER_CTRL_REG                            0xFE // Main sequencer control
#define     SEQUENCER_CTRL_PROGRAM_EN_BIT             0x0001 // Writing 1 in STANDBY/SLEEP forces the main sequencer to go to PROGRAM state. Writing 0 in PROGRAM state initiates a transition back to STANDBY/SLEEP.
#define     SEQUENCER_CTRL_MAIN_RESET_BIT             0x0002 // When 1, all non-OTP RWT registers are reset to their default value (apart from sequencer_ctrl). This includes debug registers. Note that this bit is NOT auto-cleared.
#define     SEQUENCER_CTRL_SOFT_RESET_BIT             0x8000 // Software reset (Auto Clear bit)

#define GLOBAL_LOCK_REG                               0xFF // Global lock/unlock register. This register protects all register fields of type RWT (OTP) from being written

#define OTP_CONST_SETTINGS_OTP_CONST_HEADER_REG       0x00 // CONST area header

#define OTP_CONST_SETTINGS_TRIM0_REG                  0x01 // Analog trimming
#define     OTP_CONST_SETTINGS_TRIM0_LPREF_IBIAS_MASK 0x000F // LP-reference bias current trimming (2's complement)
#define     OTP_CONST_SETTINGS_TRIM0_LPREF_IBIAS_POS  0
#define     OTP_CONST_SETTINGS_TRIM0_LPREF_VREF_MASK  0x01F0 // LP-reference reference voltage trimming (2's complement)
#define     OTP_CONST_SETTINGS_TRIM0_LPREF_VREF_POS   4

#define OTP_CONST_SETTINGS_TRIM1_REG                  0x02 // Analog trimming
#define     OTP_CONST_SETTINGS_TRIM1_BG_PTAT_MASK     0x000F // Bandgap PTAT current trimming (unsigned)
#define     OTP_CONST_SETTINGS_TRIM1_BG_PTAT_POS      0
#define     OTP_CONST_SETTINGS_TRIM1_BG_VBG_MASK      0x01F0 // Bandgap offset (2's complement)
#define     OTP_CONST_SETTINGS_TRIM1_BG_VBG_POS       4

#define OTP_CONST_SETTINGS_TRIM2_REG                  0x03 // Analog trimming
#define     OTP_CONST_SETTINGS_TRIM2_HFO_MASK         0x007F // HFO trimming (unsigned)
#define     OTP_CONST_SETTINGS_TRIM2_HFO_POS          0
#define     OTP_CONST_SETTINGS_TRIM2_LFO_MASK         0x0780 // LFO trimming (2's complement)
#define     OTP_CONST_SETTINGS_TRIM2_LFO_POS          7
#define     OTP_CONST_SETTINGS_TRIM2_LDOTMR_MASK      0x3800 // LDOTMR output voltage trimming (unsigned)
#define     OTP_CONST_SETTINGS_TRIM2_LDOTMR_POS       11

#define OTP_CONST_SETTINGS_TRIM3_REG                  0x04 // Analog/Temperature sensor trimming
#define     OTP_CONST_SETTINGS_TRIM3_DVBE_C_MASK      0x000F // Temperature sensor delta VBE coarse trim code (unsigned)
#define     OTP_CONST_SETTINGS_TRIM3_DVBE_C_POS       0
#define     OTP_CONST_SETTINGS_TRIM3_DVBE_F_MASK      0x01F0 // Temperature sensor delta VBE fine trim code (unsigned)
#define     OTP_CONST_SETTINGS_TRIM3_DVBE_F_POS       4
#define     OTP_CONST_SETTINGS_TRIM3_LDOA_MASK        0x0600 // LDOA output voltage trimming (2's complement)
#define     OTP_CONST_SETTINGS_TRIM3_LDOA_POS         9

#define OTP_CONST_SETTINGS_TRIM4_REG                  0x05 // Temperature sensor trimming
#define     OTP_CONST_SETTINGS_TRIM4_VBE_C_MASK       0x000F // Temperature sensor VBE coarse trim code (unsigned)
#define     OTP_CONST_SETTINGS_TRIM4_VBE_C_POS        0
#define     OTP_CONST_SETTINGS_TRIM4_VBE_F_MASK       0x01F0 // Temperature sensor VBE fine trim code (unsigned)
#define     OTP_CONST_SETTINGS_TRIM4_VBE_F_POS        4
#define     OTP_CONST_SETTINGS_TRIM4_SPARE_MASK       0x0C00
#define     OTP_CONST_SETTINGS_TRIM4_SPARE_POS        10
#define     OTP_CONST_SETTINGS_TRIM4_SPARE_ISO_MASK   0x3000
#define     OTP_CONST_SETTINGS_TRIM4_SPARE_ISO_POS    12
#define     OTP_CONST_SETTINGS_TRIM4_SPARELS_MASK     0xC000
#define     OTP_CONST_SETTINGS_TRIM4_SPARELS_POS      14

#define OTP_CONST_SETTINGS_TEMP_OFFSET_REG            0x06 // Temperature sensor calibration
#define     OTP_CONST_SETTINGS_TEMP_OFFSET_TEMP_OFFSET_MASK 0x01FF // Temperature sensor offset calibration
#define     OTP_CONST_SETTINGS_TEMP_OFFSET_TEMP_OFFSET_POS 0
#define     OTP_CONST_SETTINGS_TEMP_OFFSET_TEMP_DECIM_MASK 0xFE00 // Temperature sensor decimation factor calibration
#define     OTP_CONST_SETTINGS_TEMP_OFFSET_TEMP_DECIM_POS 9

#define OTP_CONST_SETTINGS_TEMP_SLOPE_REG             0x07 // Temperature sensor calibration
#define     OTP_CONST_SETTINGS_TEMP_SLOPE_TEMP_SLOPE_MASK 0x00FF // Temperature sensor slope calibration
#define     OTP_CONST_SETTINGS_TEMP_SLOPE_TEMP_SLOPE_POS 0

#define OTP_CONST_SETTINGS_AFE_CONFIGURATION_REG      0x08 // AFE configuration
#define     OTP_CONST_SETTINGS_AFE_CONFIGURATION_AFE_IA_GAIN_MASK 0x0007 // AFE IA gain selection
#define     OTP_CONST_SETTINGS_AFE_CONFIGURATION_AFE_IA_GAIN_POS 0
#define     OTP_CONST_SETTINGS_AFE_CONFIGURATION_SWAP_SIN_COS_BIT 0x0008 // Swap SIN and COS channels
#define     OTP_CONST_SETTINGS_AFE_CONFIGURATION_INV_SIN_BIT 0x0010 // Invert SIN channel
#define     OTP_CONST_SETTINGS_AFE_CONFIGURATION_INV_COS_BIT 0x0020 // Invert COS channel
#define     OTP_CONST_SETTINGS_AFE_CONFIGURATION_IA_OFFSET_MASK 0x0FC0 // IA offset trimming. MSB selects sign, offset code is unsigned.
#define     OTP_CONST_SETTINGS_AFE_CONFIGURATION_IA_OFFSET_POS 6
#define     OTP_CONST_SETTINGS_AFE_CONFIGURATION_CLK_SEL_BIT 0x1000 // Invert the clock sampling adc data before decimation

#define OTP_CONST_SETTINGS_OTP_CONST_CRC_REG          0x09 // Constant area CRC

#define OTP_TRAC_AREA_ASIC_WAFER_ID0_REG              0x0A // ASIC Wafer ID - Not mirrored

#define OTP_TRAC_AREA_ASIC_WAFER_ID1_REG              0x0B // ASIC Wafer ID - Not mirrored

#define OTP_TRAC_AREA_ASIC_WAFER_ID2_REG              0x0C // ASIC Wafer ID - Not mirrored

#define OTP_TRAC_AREA_ASIC_WAFER_ID3_REG              0x0D // ASIC Wafer ID - Not mirrored

#define OTP_TRAC_AREA_ASIC_DIE_ID0_REG                0x0E // ASIC Die ID - Not mirrored

#define OTP_TRAC_AREA_ASIC_DIE_ID1_REG                0x0F // ASIC Die ID - Not mirrored

#define OTP_TRAC_AREA_TMR_WAFER_ID0_REG               0x10 // TMR Wafer ID - Not mirrored

#define OTP_TRAC_AREA_TMR_WAFER_ID1_REG               0x11 // TMR Wafer ID - Not mirrored

#define OTP_TRAC_AREA_TMR_WAFER_ID2_REG               0x12 // TMR Wafer ID - Not mirrored

#define OTP_TRAC_AREA_TMR_DIE_ID0_REG                 0x13 // TMR Die ID - Not mirrored

#define OTP_TRAC_AREA_TMR_DIE_ID1_REG                 0x14 // TMR Die ID - Not mirrored

#define OTP_TRAC_AREA_OTP_TRAC_CRC_REG                0x15 // Tracability Area Crc - Not mirrored

#define OTP_CONF_SETTINGS0_OTP_CONF_HEADER_REG        0x16 // CONF area header

#define OTP_CONF_SETTINGS0_DSP_CONFIG_REG             0x17 // DSP configuration
#define     OTP_CONF_SETTINGS0_DSP_CONFIG_NORM_GAIN_FB_MASK 0x000F // Feedback factor for the normalization update (Unsigned 4 bit, 0 to 2-2^(-3)). Smaller values mean slower updates and higher values mean faster updates and higher suscetibility for noise. Any value >1.0 makes the system unstable.
#define     OTP_CONF_SETTINGS0_DSP_CONFIG_NORM_GAIN_FB_POS 0
#define     OTP_CONF_SETTINGS0_DSP_CONFIG_NORM_OFFS_FB_MASK 0x00F0 // Feedback factor for the normalization update (Unsigned 4 bit, 0 to 2-2^(-3)). Smaller values mean slower updates and higher values mean faster updates and higher suscetibility for noise. Any value >1.0 makes the system unstable.
#define     OTP_CONF_SETTINGS0_DSP_CONFIG_NORM_OFFS_FB_POS 4
#define     OTP_CONF_SETTINGS0_DSP_CONFIG_DISABLE_NORMALIZE_BIT 0x0100 // If set, force sin-cos normalization to static programmed values (gain_ic, offset_ic)
#define     OTP_CONF_SETTINGS0_DSP_CONFIG_LIN_MODE_BIT 0x0200 // 0: angular mode
#define     OTP_CONF_SETTINGS0_DSP_CONFIG_STARTUP_TIMER_MASK 0xC000
#define     OTP_CONF_SETTINGS0_DSP_CONFIG_STARTUP_TIMER_POS 14

#define OTP_CONF_SETTINGS0_SYS_CONFIG_REG             0x18 // System config
#define     OTP_CONF_SETTINGS0_SYS_CONFIG_PREDICT_NM_RATIO_MASK 0x03FF // Prediction N/M ratio (N is the latency in samples, M is fixed to 12). Format is 0 integer bits, 10 fractional, assumed as unsigned number. Setting the ratio to 0 disables the predictor.
#define     OTP_CONF_SETTINGS0_SYS_CONFIG_PREDICT_NM_RATIO_POS 0
#define     OTP_CONF_SETTINGS0_SYS_CONFIG_DIS_STARTUP_INT_BIT 0x0800 // Disable Startup completion interrupt
#define     OTP_CONF_SETTINGS0_SYS_CONFIG_I2C_ADDR_EN_BIT 0x1000 // I2C slave address selection method.
                                                                        // 0: I2C address based on SA0 and SA1
                                                                        // 1: I2C address based on I2C_ADDR Register

#define     OTP_CONF_SETTINGS0_SYS_CONFIG_I2C_ADDR_MASK 0x6000 // I2C_ADDR:
                                                                        // 0: 0x17
                                                                        // 1: 0x23
                                                                        // 2: 0x56
                                                                        // 3: 0x65

#define     OTP_CONF_SETTINGS0_SYS_CONFIG_I2C_ADDR_POS 13
#define     OTP_CONF_SETTINGS0_SYS_CONFIG_SLEEP_EN_BIT 0x8000 // Enable SLEEP state

#define OTP_CONF_SETTINGS0_IF_CONFIG0_REG             0x19 // Interface Settings
#define     OTP_CONF_SETTINGS0_IF_CONFIG0_SPI_MODE_BIT 0x0001 // SPI Mode:
                                                                        // 0: SPI4
                                                                        // 1: SPI3

#define     OTP_CONF_SETTINGS0_IF_CONFIG0_SPI_I2C_SEL_BIT 0x0002 // Interface Selection
                                                                        // 0: SPI Selected
                                                                        // 1: I2C Selected

#define     OTP_CONF_SETTINGS0_IF_CONFIG0_ABZ_EN_BIT  0x0004 // Enable ABZ interface
#define     OTP_CONF_SETTINGS0_IF_CONFIG0_ABZ_STEP_MODE_BIT 0x0008 // ABZ: Step direction mode
#define     OTP_CONF_SETTINGS0_IF_CONFIG0_ABZ_STARTUP_BIT 0x0010 // ABZ: Increment from zero to current angle at activation
#define     OTP_CONF_SETTINGS0_IF_CONFIG0_ABZ_Z_PULSE_WIDTH_MASK 0x03E0 // ABZ: Duration of Z pulse in 2us steps. Code 0 = 2us, code 31=64us
#define     OTP_CONF_SETTINGS0_IF_CONFIG0_ABZ_Z_PULSE_WIDTH_POS 5
#define     OTP_CONF_SETTINGS0_IF_CONFIG0_ABZ_AB_MIN_PULSE_WIDTH_MASK 0x3C00 // ABZ: Minimum Pulse width settings for the A and B outputs
                                                                        // 0: 0.125?s
                                                                        // 1: 0.25?s
                                                                        // 2: 0.50?s
                                                                        // 3: 0.75?s
                                                                        // 4: 1.00?s
                                                                        // 5: 1.25?s
                                                                        // 6: 1.50?s
                                                                        // 7: 1.75?s
                                                                        // 8: 2.00?S
                                                                        // 9: 2.25?S
                                                                        // 10: 2.50?S
                                                                        // 11: 2.75?S
                                                                        // 12: 3.00?S
                                                                        // 13: 3.25?s
                                                                        // 14: 3.50?S
                                                                        // 15: 3.75?S

#define     OTP_CONF_SETTINGS0_IF_CONFIG0_ABZ_AB_MIN_PULSE_WIDTH_POS 10
#define     OTP_CONF_SETTINGS0_IF_CONFIG0_ABZ_Z_PULSE_CONFIG_MASK 0xC000 // ABZ: Z pulse mode configuration
                                                                        // 0: 1LSB X4-Decoding
                                                                        // 1: 3LSB X4-Decoding
                                                                        // 2: Fixed Duration (Together with abz_z_pulse_width setting
                                                                        // 3: 180DEG Mode

#define     OTP_CONF_SETTINGS0_IF_CONFIG0_ABZ_Z_PULSE_CONFIG_POS 14

#define OTP_CONF_SETTINGS0_IF_CONFIG1_REG             0x1A // Interface Settings
#define     OTP_CONF_SETTINGS0_IF_CONFIG1_ABZ_PPR_MASK 0x0FFF // ABZ: pulses per rotation (PPR = register value+1)
#define     OTP_CONF_SETTINGS0_IF_CONFIG1_ABZ_PPR_POS 0
#define     OTP_CONF_SETTINGS0_IF_CONFIG1_CSB_LOW_DETECT_ENABLE_BIT 0x1000 // Enable the pulldown detection of the CSB line to exit ABZ mode
#define     OTP_CONF_SETTINGS0_IF_CONFIG1_SPI_DSP_SAMPLE_MARGIN_MASK 0xE000 // Sets the nr. of sclk periods before the end of the stream command, to sample the dsp_angle_value
                                                                        // 0: 3 sclk periods
                                                                        // 1: 1 sclk periods
                                                                        // 2: 2 sclk periods
                                                                        // 3: 3 sclk periods
                                                                        // 4: 4 sclk periods
                                                                        // 5: 5 sclk periods
                                                                        // 6: resv
                                                                        // 7: resv

#define     OTP_CONF_SETTINGS0_IF_CONFIG1_SPI_DSP_SAMPLE_MARGIN_POS 13

#define OTP_CONF_SETTINGS0_IF_CONFIG2_REG             0x1B // Interface Settings
#define     OTP_CONF_SETTINGS0_IF_CONFIG2_SPI_I2C_DRV_MASK 0x0003 // SPI and I2C IO Drive Strength:
                                                                        // 0: 1mA
                                                                        // 1: 2mA
                                                                        // 2: 4mA
                                                                        // 3: 8mA

#define     OTP_CONF_SETTINGS0_IF_CONFIG2_SPI_I2C_DRV_POS 0
#define     OTP_CONF_SETTINGS0_IF_CONFIG2_INT_DRV_MASK 0x000C // Int IO drive Strength:
                                                                        // 0: 1mA
                                                                        // 1: 2mA
                                                                        // 2: 4mA
                                                                        // 3: 8mA

#define     OTP_CONF_SETTINGS0_IF_CONFIG2_INT_DRV_POS 2
#define     OTP_CONF_SETTINGS0_IF_CONFIG2_ABZ_DRV_MASK 0x0030 // abz IO drive Strength:
                                                                        // 0: 1mA
                                                                        // 1: 2mA
                                                                        // 2: 4mA
                                                                        // 3: 8mA

#define     OTP_CONF_SETTINGS0_IF_CONFIG2_ABZ_DRV_POS 4
#define     OTP_CONF_SETTINGS0_IF_CONFIG2_INT_PU_DIS_BIT 0x0040 // Disable weak pullup on INT pad
#define     OTP_CONF_SETTINGS0_IF_CONFIG2_SCLK_PU_DIS_BIT 0x0080 // Disable weak pullup on SCLK pad
#define     OTP_CONF_SETTINGS0_IF_CONFIG2_CSB_PU_DIS_BIT 0x0100 // Disable weak pullup on CSB pad
#define     OTP_CONF_SETTINGS0_IF_CONFIG2_MOSI_PU_EN_BIT 0x0200 // Enable weak pullup on MOSI pad
#define     OTP_CONF_SETTINGS0_IF_CONFIG2_HYST_ENABLE_BIT 0x0400 // Activate Hysteresis on ABZ interface
#define     OTP_CONF_SETTINGS0_IF_CONFIG2_HYST_HYST_MASK 0x1800 // Hysteresis:
                                                                        // 0: 4 LSB Hysteresis
                                                                        // 1: 8 LSB Hysteresis
                                                                        // 2: 16 LSB Hysteresis
                                                                        // 3: 32 LSB Hysteresis

#define     OTP_CONF_SETTINGS0_IF_CONFIG2_HYST_HYST_POS 11
#define     OTP_CONF_SETTINGS0_IF_CONFIG2_HYST_OFFSET_CONFIG_MASK 0x6000 // Hyst: Enable/Disable the catch-up (Disabling introduces an offset in CW or CCW direction)
                                                                        // 0: Hysteresis Catch-up Enabled
                                                                        // 1: Hysteresis Catch-up Disabled, Offset in CW Direction
                                                                        // 2: Hysteresis Catch-up Disabled, Offset in CCW Direction
                                                                        // 3: Hysteresis Catch-up Enabled

#define     OTP_CONF_SETTINGS0_IF_CONFIG2_HYST_OFFSET_CONFIG_POS 13

#define OTP_CONF_SETTINGS0_OTP_CONF_CRC_REG           0x1C // CONF area CRC

#define OTP_CONF_SETTINGS1_OTP_CONF_HEADER_REG        0x1D // CONF area header

#define OTP_CONF_SETTINGS1_DSP_CONFIG_REG             0x1E // DSP configuration
#define     OTP_CONF_SETTINGS1_DSP_CONFIG_NORM_GAIN_FB_MASK 0x000F // Feedback factor for the normalization update (Unsigned 4 bit, 0 to 2-2^(-3)). Smaller values mean slower updates and higher values mean faster updates and higher suscetibility for noise. Any value >1.0 makes the system unstable.
#define     OTP_CONF_SETTINGS1_DSP_CONFIG_NORM_GAIN_FB_POS 0
#define     OTP_CONF_SETTINGS1_DSP_CONFIG_NORM_OFFS_FB_MASK 0x00F0 // Feedback factor for the normalization update (Unsigned 4 bit, 0 to 2-2^(-3)). Smaller values mean slower updates and higher values mean faster updates and higher suscetibility for noise. Any value >1.0 makes the system unstable.
#define     OTP_CONF_SETTINGS1_DSP_CONFIG_NORM_OFFS_FB_POS 4
#define     OTP_CONF_SETTINGS1_DSP_CONFIG_DISABLE_NORMALIZE_BIT 0x0100 // If set, force sin-cos normalization to static programmed values (gain_ic, offset_ic)
#define     OTP_CONF_SETTINGS1_DSP_CONFIG_LIN_MODE_BIT 0x0200 // 0: angular mode
#define     OTP_CONF_SETTINGS1_DSP_CONFIG_STARTUP_TIMER_MASK 0xC000
#define     OTP_CONF_SETTINGS1_DSP_CONFIG_STARTUP_TIMER_POS 14

#define OTP_CONF_SETTINGS1_SYS_CONFIG_REG             0x1F // System config
#define     OTP_CONF_SETTINGS1_SYS_CONFIG_PREDICT_NM_RATIO_MASK 0x03FF // Prediction N/M ratio (N is the latency in samples, M is fixed to 12). Format is 0 integer bits, 10 fractional, assumed as unsigned number. Setting the ratio to 0 disables the predictor.
#define     OTP_CONF_SETTINGS1_SYS_CONFIG_PREDICT_NM_RATIO_POS 0
#define     OTP_CONF_SETTINGS1_SYS_CONFIG_DIS_STARTUP_INT_BIT 0x0800 // Disable Startup completion interrupt
#define     OTP_CONF_SETTINGS1_SYS_CONFIG_I2C_ADDR_EN_BIT 0x1000 // I2C slave address selection method.
                                                                        // 0: I2C address based on SA0 and SA1
                                                                        // 1: I2C address based on I2C_ADDR Register

#define     OTP_CONF_SETTINGS1_SYS_CONFIG_I2C_ADDR_MASK 0x6000 // I2C_ADDR:
                                                                        // 0: 0x17
                                                                        // 1: 0x23
                                                                        // 2: 0x56
                                                                        // 3: 0x65

#define     OTP_CONF_SETTINGS1_SYS_CONFIG_I2C_ADDR_POS 13
#define     OTP_CONF_SETTINGS1_SYS_CONFIG_SLEEP_EN_BIT 0x8000 // Enable SLEEP state

#define OTP_CONF_SETTINGS1_IF_CONFIG0_REG             0x20 // Interface Settings
#define     OTP_CONF_SETTINGS1_IF_CONFIG0_SPI_MODE_BIT 0x0001 // SPI Mode:
                                                                        // 0: SPI4
                                                                        // 1: SPI3

#define     OTP_CONF_SETTINGS1_IF_CONFIG0_SPI_I2C_SEL_BIT 0x0002 // Interface Selection
                                                                        // 0: SPI Selected
                                                                        // 1: I2C Selected

#define     OTP_CONF_SETTINGS1_IF_CONFIG0_ABZ_EN_BIT  0x0004 // Enable ABZ interface
#define     OTP_CONF_SETTINGS1_IF_CONFIG0_ABZ_STEP_MODE_BIT 0x0008 // ABZ: Step direction mode
#define     OTP_CONF_SETTINGS1_IF_CONFIG0_ABZ_STARTUP_BIT 0x0010 // ABZ: Increment from zero to current angle at activation
#define     OTP_CONF_SETTINGS1_IF_CONFIG0_ABZ_Z_PULSE_WIDTH_MASK 0x03E0 // ABZ: Duration of Z pulse in 2us steps. Code 0 = 2us, code 31=64us
#define     OTP_CONF_SETTINGS1_IF_CONFIG0_ABZ_Z_PULSE_WIDTH_POS 5
#define     OTP_CONF_SETTINGS1_IF_CONFIG0_ABZ_AB_MIN_PULSE_WIDTH_MASK 0x3C00 // ABZ: Minimum Pulse width settings for the A and B outputs
                                                                        // 0: 0.125?s
                                                                        // 1: 0.25?s
                                                                        // 2: 0.50?s
                                                                        // 3: 0.75?s
                                                                        // 4: 1.00?s
                                                                        // 5: 1.25?s
                                                                        // 6: 1.50?s
                                                                        // 7: 1.75?s
                                                                        // 8: 2.00?S
                                                                        // 9: 2.25?S
                                                                        // 10: 2.50?S
                                                                        // 11: 2.75?S
                                                                        // 12: 3.00?S
                                                                        // 13: 3.25?s
                                                                        // 14: 3.50?S
                                                                        // 15: 3.75?S

#define     OTP_CONF_SETTINGS1_IF_CONFIG0_ABZ_AB_MIN_PULSE_WIDTH_POS 10
#define     OTP_CONF_SETTINGS1_IF_CONFIG0_ABZ_Z_PULSE_CONFIG_MASK 0xC000 // ABZ: Z pulse mode configuration
                                                                        // 0: 1LSB X4-Decoding
                                                                        // 1: 3LSB X4-Decoding
                                                                        // 2: Fixed Duration (Together with abz_z_pulse_width setting
                                                                        // 3: 180DEG Mode

#define     OTP_CONF_SETTINGS1_IF_CONFIG0_ABZ_Z_PULSE_CONFIG_POS 14

#define OTP_CONF_SETTINGS1_IF_CONFIG1_REG             0x21 // Interface Settings
#define     OTP_CONF_SETTINGS1_IF_CONFIG1_ABZ_PPR_MASK 0x0FFF // ABZ: pulses per rotation (PPR = register value+1)
#define     OTP_CONF_SETTINGS1_IF_CONFIG1_ABZ_PPR_POS 0
#define     OTP_CONF_SETTINGS1_IF_CONFIG1_CSB_LOW_DETECT_ENABLE_BIT 0x1000 // Enable the pulldown detection of the CSB line to exit ABZ mode
#define     OTP_CONF_SETTINGS1_IF_CONFIG1_SPI_DSP_SAMPLE_MARGIN_MASK 0xE000 // Sets the nr. of sclk periods before the end of the stream command, to sample the dsp_angle_value
                                                                        // 0: 3 sclk periods
                                                                        // 1: 1 sclk periods
                                                                        // 2: 2 sclk periods
                                                                        // 3: 3 sclk periods
                                                                        // 4: 4 sclk periods
                                                                        // 5: 5 sclk periods
                                                                        // 6: resv
                                                                        // 7: resv

#define     OTP_CONF_SETTINGS1_IF_CONFIG1_SPI_DSP_SAMPLE_MARGIN_POS 13

#define OTP_CONF_SETTINGS1_IF_CONFIG2_REG             0x22 // Interface Settings
#define     OTP_CONF_SETTINGS1_IF_CONFIG2_SPI_I2C_DRV_MASK 0x0003 // SPI and I2C IO Drive Strength:
                                                                        // 0: 1mA
                                                                        // 1: 2mA
                                                                        // 2: 4mA
                                                                        // 3: 8mA

#define     OTP_CONF_SETTINGS1_IF_CONFIG2_SPI_I2C_DRV_POS 0
#define     OTP_CONF_SETTINGS1_IF_CONFIG2_INT_DRV_MASK 0x000C // Int IO drive Strength:
                                                                        // 0: 1mA
                                                                        // 1: 2mA
                                                                        // 2: 4mA
                                                                        // 3: 8mA

#define     OTP_CONF_SETTINGS1_IF_CONFIG2_INT_DRV_POS 2
#define     OTP_CONF_SETTINGS1_IF_CONFIG2_ABZ_DRV_MASK 0x0030 // abz IO drive Strength:
                                                                        // 0: 1mA
                                                                        // 1: 2mA
                                                                        // 2: 4mA
                                                                        // 3: 8mA

#define     OTP_CONF_SETTINGS1_IF_CONFIG2_ABZ_DRV_POS 4
#define     OTP_CONF_SETTINGS1_IF_CONFIG2_INT_PU_DIS_BIT 0x0040 // Disable weak pullup on INT pad
#define     OTP_CONF_SETTINGS1_IF_CONFIG2_SCLK_PU_DIS_BIT 0x0080 // Disable weak pullup on SCLK pad
#define     OTP_CONF_SETTINGS1_IF_CONFIG2_CSB_PU_DIS_BIT 0x0100 // Disable weak pullup on CSB pad
#define     OTP_CONF_SETTINGS1_IF_CONFIG2_MOSI_PU_EN_BIT 0x0200 // Enable weak pullup on MOSI pad
#define     OTP_CONF_SETTINGS1_IF_CONFIG2_HYST_ENABLE_BIT 0x0400 // Activate Hysteresis on ABZ interface
#define     OTP_CONF_SETTINGS1_IF_CONFIG2_HYST_HYST_MASK 0x1800 // Hysteresis:
                                                                        // 0: 4 LSB Hysteresis
                                                                        // 1: 8 LSB Hysteresis
                                                                        // 2: 16 LSB Hysteresis
                                                                        // 3: 32 LSB Hysteresis

#define     OTP_CONF_SETTINGS1_IF_CONFIG2_HYST_HYST_POS 11
#define     OTP_CONF_SETTINGS1_IF_CONFIG2_HYST_OFFSET_CONFIG_MASK 0x6000 // Hyst: Enable/Disable the catch-up (Disabling introduces an offset in CW or CCW direction)
                                                                        // 0: Hysteresis Catch-up Enabled
                                                                        // 1: Hysteresis Catch-up Disabled, Offset in CW Direction
                                                                        // 2: Hysteresis Catch-up Disabled, Offset in CCW Direction
                                                                        // 3: Hysteresis Catch-up Enabled

#define     OTP_CONF_SETTINGS1_IF_CONFIG2_HYST_OFFSET_CONFIG_POS 13

#define OTP_CONF_SETTINGS1_OTP_CONF_CRC_REG           0x23 // CONF area CRC

#define OTP_CALIB_SETTINGS0_OTP_CALIB_HEADER_REG      0x24 // CALIB area header

#define OTP_CALIB_SETTINGS0_LUT_OUT0_REG              0x25 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT1_REG              0x26 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT2_REG              0x27 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT3_REG              0x28 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT4_REG              0x29 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT5_REG              0x2A // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT6_REG              0x2B // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT7_REG              0x2C // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT8_REG              0x2D // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT9_REG              0x2E // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT10_REG             0x2F // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT11_REG             0x30 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT12_REG             0x31 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT13_REG             0x32 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT14_REG             0x33 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT15_REG             0x34 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT16_REG             0x35 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT17_REG             0x36 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT18_REG             0x37 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT19_REG             0x38 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT20_REG             0x39 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT21_REG             0x3A // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT22_REG             0x3B // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT23_REG             0x3C // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT24_REG             0x3D // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT25_REG             0x3E // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT26_REG             0x3F // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT27_REG             0x40 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT28_REG             0x41 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT29_REG             0x42 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT30_REG             0x43 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_LUT_OUT31_REG             0x44 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS0_SIN_LIN_GAIN_IC_REG       0x45 // Gain initial condition for the normalize block

#define OTP_CALIB_SETTINGS0_COS_GAIN_IC_REG           0x46 // Gain initial condition for the normalize block

#define OTP_CALIB_SETTINGS0_SIN_LIN_OFFS_IC_REG       0x47 // Offset initial condition for the normalize block

#define OTP_CALIB_SETTINGS0_COS_OFFS_IC_REG           0x48 // Offset initial condition for the normalize block

#define OTP_CALIB_SETTINGS0_SIN_GAIN_TC_REG           0x49 // Sin/Lin channel gain temperature slope
#define     OTP_CALIB_SETTINGS0_SIN_GAIN_TC_GAIN_TC_MASK 0x0FFF // Sin/Lin channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     OTP_CALIB_SETTINGS0_SIN_GAIN_TC_GAIN_TC_POS 0
#define     OTP_CALIB_SETTINGS0_SIN_GAIN_TC_TREF_3_0_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     OTP_CALIB_SETTINGS0_SIN_GAIN_TC_TREF_3_0_POS 12

#define OTP_CALIB_SETTINGS0_SIN_OFFS_TC_REG           0x4A // Sin/Lin channel offset temperature slope
#define     OTP_CALIB_SETTINGS0_SIN_OFFS_TC_OFFS_TC_MASK 0x0FFF // Sin/Lin channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     OTP_CALIB_SETTINGS0_SIN_OFFS_TC_OFFS_TC_POS 0
#define     OTP_CALIB_SETTINGS0_SIN_OFFS_TC_TREF_7_4_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [7:4
#define     OTP_CALIB_SETTINGS0_SIN_OFFS_TC_TREF_7_4_POS 12

#define OTP_CALIB_SETTINGS0_COS_GAIN_TC_REG           0x4B // Cos channel gain temperature slope
#define     OTP_CALIB_SETTINGS0_COS_GAIN_TC_GAIN_TC_MASK 0x0FFF // Cos channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     OTP_CALIB_SETTINGS0_COS_GAIN_TC_GAIN_TC_POS 0
#define     OTP_CALIB_SETTINGS0_COS_GAIN_TC_TREF_11_8_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     OTP_CALIB_SETTINGS0_COS_GAIN_TC_TREF_11_8_POS 12

#define OTP_CALIB_SETTINGS0_COS_OFFS_TC_REG           0x4C // Cos channel offset temperature slope
#define     OTP_CALIB_SETTINGS0_COS_OFFS_TC_OFFS_TC_MASK 0x0FFF // Cos channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     OTP_CALIB_SETTINGS0_COS_OFFS_TC_OFFS_TC_POS 0
#define     OTP_CALIB_SETTINGS0_COS_OFFS_TC_TREF_12_BIT 0x1000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bit [12

#define OTP_CALIB_SETTINGS0_OUT_ADJ_REG               0x4D // Output adjustment settings
#define     OTP_CALIB_SETTINGS0_OUT_ADJ_OFFSET_MASK   0x7FFF // Output angle offset (LSB=360*2^-15 deg)
#define     OTP_CALIB_SETTINGS0_OUT_ADJ_OFFSET_POS    0
#define     OTP_CALIB_SETTINGS0_OUT_ADJ_CW_CCW_SIGN_BIT 0x8000 // CW/CCW output adjustment:
                                                                        // 0: CCW No output adjustment
                                                                        // 1: CW (360 - measure angle)


#define OTP_CALIB_SETTINGS0_OTP_CALIB_CRC_REG         0x4E // CALIB area CRC

#define OTP_CALIB_SETTINGS1_OTP_CALIB_HEADER_REG      0x4F // CALIB area header

#define OTP_CALIB_SETTINGS1_LUT_OUT0_REG              0x50 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT1_REG              0x51 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT2_REG              0x52 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT3_REG              0x53 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT4_REG              0x54 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT5_REG              0x55 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT6_REG              0x56 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT7_REG              0x57 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT8_REG              0x58 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT9_REG              0x59 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT10_REG             0x5A // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT11_REG             0x5B // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT12_REG             0x5C // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT13_REG             0x5D // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT14_REG             0x5E // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT15_REG             0x5F // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT16_REG             0x60 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT17_REG             0x61 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT18_REG             0x62 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT19_REG             0x63 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT20_REG             0x64 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT21_REG             0x65 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT22_REG             0x66 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT23_REG             0x67 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT24_REG             0x68 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT25_REG             0x69 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT26_REG             0x6A // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT27_REG             0x6B // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT28_REG             0x6C // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT29_REG             0x6D // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT30_REG             0x6E // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_LUT_OUT31_REG             0x6F // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS1_SIN_LIN_GAIN_IC_REG       0x70 // Gain initial condition for the normalize block

#define OTP_CALIB_SETTINGS1_COS_GAIN_IC_REG           0x71 // Gain initial condition for the normalize block

#define OTP_CALIB_SETTINGS1_SIN_LIN_OFFS_IC_REG       0x72 // Offset initial condition for the normalize block

#define OTP_CALIB_SETTINGS1_COS_OFFS_IC_REG           0x73 // Offset initial condition for the normalize block

#define OTP_CALIB_SETTINGS1_SIN_GAIN_TC_REG           0x74 // Sin/Lin channel gain temperature slope
#define     OTP_CALIB_SETTINGS1_SIN_GAIN_TC_GAIN_TC_MASK 0x0FFF // Sin/Lin channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     OTP_CALIB_SETTINGS1_SIN_GAIN_TC_GAIN_TC_POS 0
#define     OTP_CALIB_SETTINGS1_SIN_GAIN_TC_TREF_3_0_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     OTP_CALIB_SETTINGS1_SIN_GAIN_TC_TREF_3_0_POS 12

#define OTP_CALIB_SETTINGS1_SIN_OFFS_TC_REG           0x75 // Sin/Lin channel offset temperature slope
#define     OTP_CALIB_SETTINGS1_SIN_OFFS_TC_OFFS_TC_MASK 0x0FFF // Sin/Lin channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     OTP_CALIB_SETTINGS1_SIN_OFFS_TC_OFFS_TC_POS 0
#define     OTP_CALIB_SETTINGS1_SIN_OFFS_TC_TREF_7_4_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [7:4
#define     OTP_CALIB_SETTINGS1_SIN_OFFS_TC_TREF_7_4_POS 12

#define OTP_CALIB_SETTINGS1_COS_GAIN_TC_REG           0x76 // Cos channel gain temperature slope
#define     OTP_CALIB_SETTINGS1_COS_GAIN_TC_GAIN_TC_MASK 0x0FFF // Cos channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     OTP_CALIB_SETTINGS1_COS_GAIN_TC_GAIN_TC_POS 0
#define     OTP_CALIB_SETTINGS1_COS_GAIN_TC_TREF_11_8_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     OTP_CALIB_SETTINGS1_COS_GAIN_TC_TREF_11_8_POS 12

#define OTP_CALIB_SETTINGS1_COS_OFFS_TC_REG           0x77 // Cos channel offset temperature slope
#define     OTP_CALIB_SETTINGS1_COS_OFFS_TC_OFFS_TC_MASK 0x0FFF // Cos channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     OTP_CALIB_SETTINGS1_COS_OFFS_TC_OFFS_TC_POS 0
#define     OTP_CALIB_SETTINGS1_COS_OFFS_TC_TREF_12_BIT 0x1000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bit [12

#define OTP_CALIB_SETTINGS1_OUT_ADJ_REG               0x78 // Output adjustment settings
#define     OTP_CALIB_SETTINGS1_OUT_ADJ_OFFSET_MASK   0x7FFF // Output angle offset (LSB=360*2^-15 deg)
#define     OTP_CALIB_SETTINGS1_OUT_ADJ_OFFSET_POS    0
#define     OTP_CALIB_SETTINGS1_OUT_ADJ_CW_CCW_SIGN_BIT 0x8000 // CW/CCW output adjustment:
                                                                        // 0: CCW No output adjustment
                                                                        // 1: CW (360 - measure angle)


#define OTP_CALIB_SETTINGS1_OTP_CALIB_CRC_REG         0x79 // CALIB area CRC

#define OTP_CALIB_SETTINGS2_OTP_CALIB_HEADER_REG      0x7A // CALIB area header

#define OTP_CALIB_SETTINGS2_LUT_OUT0_REG              0x7B // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT1_REG              0x7C // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT2_REG              0x7D // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT3_REG              0x7E // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT4_REG              0x7F // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT5_REG              0x80 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT6_REG              0x81 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT7_REG              0x82 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT8_REG              0x83 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT9_REG              0x84 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT10_REG             0x85 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT11_REG             0x86 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT12_REG             0x87 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT13_REG             0x88 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT14_REG             0x89 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT15_REG             0x8A // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT16_REG             0x8B // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT17_REG             0x8C // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT18_REG             0x8D // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT19_REG             0x8E // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT20_REG             0x8F // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT21_REG             0x90 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT22_REG             0x91 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT23_REG             0x92 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT24_REG             0x93 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT25_REG             0x94 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT26_REG             0x95 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT27_REG             0x96 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT28_REG             0x97 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT29_REG             0x98 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT30_REG             0x99 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_LUT_OUT31_REG             0x9A // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS2_SIN_LIN_GAIN_IC_REG       0x9B // Gain initial condition for the normalize block

#define OTP_CALIB_SETTINGS2_COS_GAIN_IC_REG           0x9C // Gain initial condition for the normalize block

#define OTP_CALIB_SETTINGS2_SIN_LIN_OFFS_IC_REG       0x9D // Offset initial condition for the normalize block

#define OTP_CALIB_SETTINGS2_COS_OFFS_IC_REG           0x9E // Offset initial condition for the normalize block

#define OTP_CALIB_SETTINGS2_SIN_GAIN_TC_REG           0x9F // Sin/Lin channel gain temperature slope
#define     OTP_CALIB_SETTINGS2_SIN_GAIN_TC_GAIN_TC_MASK 0x0FFF // Sin/Lin channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     OTP_CALIB_SETTINGS2_SIN_GAIN_TC_GAIN_TC_POS 0
#define     OTP_CALIB_SETTINGS2_SIN_GAIN_TC_TREF_3_0_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     OTP_CALIB_SETTINGS2_SIN_GAIN_TC_TREF_3_0_POS 12

#define OTP_CALIB_SETTINGS2_SIN_OFFS_TC_REG           0xA0 // Sin/Lin channel offset temperature slope
#define     OTP_CALIB_SETTINGS2_SIN_OFFS_TC_OFFS_TC_MASK 0x0FFF // Sin/Lin channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     OTP_CALIB_SETTINGS2_SIN_OFFS_TC_OFFS_TC_POS 0
#define     OTP_CALIB_SETTINGS2_SIN_OFFS_TC_TREF_7_4_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [7:4
#define     OTP_CALIB_SETTINGS2_SIN_OFFS_TC_TREF_7_4_POS 12

#define OTP_CALIB_SETTINGS2_COS_GAIN_TC_REG           0xA1 // Cos channel gain temperature slope
#define     OTP_CALIB_SETTINGS2_COS_GAIN_TC_GAIN_TC_MASK 0x0FFF // Cos channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     OTP_CALIB_SETTINGS2_COS_GAIN_TC_GAIN_TC_POS 0
#define     OTP_CALIB_SETTINGS2_COS_GAIN_TC_TREF_11_8_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     OTP_CALIB_SETTINGS2_COS_GAIN_TC_TREF_11_8_POS 12

#define OTP_CALIB_SETTINGS2_COS_OFFS_TC_REG           0xA2 // Cos channel offset temperature slope
#define     OTP_CALIB_SETTINGS2_COS_OFFS_TC_OFFS_TC_MASK 0x0FFF // Cos channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     OTP_CALIB_SETTINGS2_COS_OFFS_TC_OFFS_TC_POS 0
#define     OTP_CALIB_SETTINGS2_COS_OFFS_TC_TREF_12_BIT 0x1000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bit [12

#define OTP_CALIB_SETTINGS2_OUT_ADJ_REG               0xA3 // Output adjustment settings
#define     OTP_CALIB_SETTINGS2_OUT_ADJ_OFFSET_MASK   0x7FFF // Output angle offset (LSB=360*2^-15 deg)
#define     OTP_CALIB_SETTINGS2_OUT_ADJ_OFFSET_POS    0
#define     OTP_CALIB_SETTINGS2_OUT_ADJ_CW_CCW_SIGN_BIT 0x8000 // CW/CCW output adjustment:
                                                                        // 0: CCW No output adjustment
                                                                        // 1: CW (360 - measure angle)


#define OTP_CALIB_SETTINGS2_OTP_CALIB_CRC_REG         0xA4 // CALIB area CRC

#define OTP_CALIB_SETTINGS3_OTP_CALIB_HEADER_REG      0xA5 // CALIB area header

#define OTP_CALIB_SETTINGS3_LUT_OUT0_REG              0xA6 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT1_REG              0xA7 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT2_REG              0xA8 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT3_REG              0xA9 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT4_REG              0xAA // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT5_REG              0xAB // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT6_REG              0xAC // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT7_REG              0xAD // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT8_REG              0xAE // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT9_REG              0xAF // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT10_REG             0xB0 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT11_REG             0xB1 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT12_REG             0xB2 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT13_REG             0xB3 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT14_REG             0xB4 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT15_REG             0xB5 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT16_REG             0xB6 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT17_REG             0xB7 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT18_REG             0xB8 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT19_REG             0xB9 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT20_REG             0xBA // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT21_REG             0xBB // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT22_REG             0xBC // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT23_REG             0xBD // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT24_REG             0xBE // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT25_REG             0xBF // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT26_REG             0xC0 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT27_REG             0xC1 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT28_REG             0xC2 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT29_REG             0xC3 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT30_REG             0xC4 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_LUT_OUT31_REG             0xC5 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS3_SIN_LIN_GAIN_IC_REG       0xC6 // Gain initial condition for the normalize block

#define OTP_CALIB_SETTINGS3_COS_GAIN_IC_REG           0xC7 // Gain initial condition for the normalize block

#define OTP_CALIB_SETTINGS3_SIN_LIN_OFFS_IC_REG       0xC8 // Offset initial condition for the normalize block

#define OTP_CALIB_SETTINGS3_COS_OFFS_IC_REG           0xC9 // Offset initial condition for the normalize block

#define OTP_CALIB_SETTINGS3_SIN_GAIN_TC_REG           0xCA // Sin/Lin channel gain temperature slope
#define     OTP_CALIB_SETTINGS3_SIN_GAIN_TC_GAIN_TC_MASK 0x0FFF // Sin/Lin channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     OTP_CALIB_SETTINGS3_SIN_GAIN_TC_GAIN_TC_POS 0
#define     OTP_CALIB_SETTINGS3_SIN_GAIN_TC_TREF_3_0_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     OTP_CALIB_SETTINGS3_SIN_GAIN_TC_TREF_3_0_POS 12

#define OTP_CALIB_SETTINGS3_SIN_OFFS_TC_REG           0xCB // Sin/Lin channel offset temperature slope
#define     OTP_CALIB_SETTINGS3_SIN_OFFS_TC_OFFS_TC_MASK 0x0FFF // Sin/Lin channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     OTP_CALIB_SETTINGS3_SIN_OFFS_TC_OFFS_TC_POS 0
#define     OTP_CALIB_SETTINGS3_SIN_OFFS_TC_TREF_7_4_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [7:4
#define     OTP_CALIB_SETTINGS3_SIN_OFFS_TC_TREF_7_4_POS 12

#define OTP_CALIB_SETTINGS3_COS_GAIN_TC_REG           0xCC // Cos channel gain temperature slope
#define     OTP_CALIB_SETTINGS3_COS_GAIN_TC_GAIN_TC_MASK 0x0FFF // Cos channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     OTP_CALIB_SETTINGS3_COS_GAIN_TC_GAIN_TC_POS 0
#define     OTP_CALIB_SETTINGS3_COS_GAIN_TC_TREF_11_8_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     OTP_CALIB_SETTINGS3_COS_GAIN_TC_TREF_11_8_POS 12

#define OTP_CALIB_SETTINGS3_COS_OFFS_TC_REG           0xCD // Cos channel offset temperature slope
#define     OTP_CALIB_SETTINGS3_COS_OFFS_TC_OFFS_TC_MASK 0x0FFF // Cos channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     OTP_CALIB_SETTINGS3_COS_OFFS_TC_OFFS_TC_POS 0
#define     OTP_CALIB_SETTINGS3_COS_OFFS_TC_TREF_12_BIT 0x1000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bit [12

#define OTP_CALIB_SETTINGS3_OUT_ADJ_REG               0xCE // Output adjustment settings
#define     OTP_CALIB_SETTINGS3_OUT_ADJ_OFFSET_MASK   0x7FFF // Output angle offset (LSB=360*2^-15 deg)
#define     OTP_CALIB_SETTINGS3_OUT_ADJ_OFFSET_POS    0
#define     OTP_CALIB_SETTINGS3_OUT_ADJ_CW_CCW_SIGN_BIT 0x8000 // CW/CCW output adjustment:
                                                                        // 0: CCW No output adjustment
                                                                        // 1: CW (360 - measure angle)


#define OTP_CALIB_SETTINGS3_OTP_CALIB_CRC_REG         0xCF // CALIB area CRC

#define OTP_CALIB_SETTINGS4_OTP_CALIB_HEADER_REG      0xD0 // CALIB area header

#define OTP_CALIB_SETTINGS4_LUT_OUT0_REG              0xD1 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT1_REG              0xD2 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT2_REG              0xD3 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT3_REG              0xD4 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT4_REG              0xD5 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT5_REG              0xD6 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT6_REG              0xD7 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT7_REG              0xD8 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT8_REG              0xD9 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT9_REG              0xDA // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT10_REG             0xDB // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT11_REG             0xDC // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT12_REG             0xDD // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT13_REG             0xDE // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT14_REG             0xDF // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT15_REG             0xE0 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT16_REG             0xE1 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT17_REG             0xE2 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT18_REG             0xE3 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT19_REG             0xE4 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT20_REG             0xE5 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT21_REG             0xE6 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT22_REG             0xE7 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT23_REG             0xE8 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT24_REG             0xE9 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT25_REG             0xEA // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT26_REG             0xEB // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT27_REG             0xEC // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT28_REG             0xED // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT29_REG             0xEE // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT30_REG             0xEF // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_LUT_OUT31_REG             0xF0 // LUT output point (32 input points are mapped to 32 output points)

#define OTP_CALIB_SETTINGS4_SIN_LIN_GAIN_IC_REG       0xF1 // Gain initial condition for the normalize block

#define OTP_CALIB_SETTINGS4_COS_GAIN_IC_REG           0xF2 // Gain initial condition for the normalize block

#define OTP_CALIB_SETTINGS4_SIN_LIN_OFFS_IC_REG       0xF3 // Offset initial condition for the normalize block

#define OTP_CALIB_SETTINGS4_COS_OFFS_IC_REG           0xF4 // Offset initial condition for the normalize block

#define OTP_CALIB_SETTINGS4_SIN_GAIN_TC_REG           0xF5 // Sin/Lin channel gain temperature slope
#define     OTP_CALIB_SETTINGS4_SIN_GAIN_TC_GAIN_TC_MASK 0x0FFF // Sin/Lin channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     OTP_CALIB_SETTINGS4_SIN_GAIN_TC_GAIN_TC_POS 0
#define     OTP_CALIB_SETTINGS4_SIN_GAIN_TC_TREF_3_0_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     OTP_CALIB_SETTINGS4_SIN_GAIN_TC_TREF_3_0_POS 12

#define OTP_CALIB_SETTINGS4_SIN_OFFS_TC_REG           0xF6 // Sin/Lin channel offset temperature slope
#define     OTP_CALIB_SETTINGS4_SIN_OFFS_TC_OFFS_TC_MASK 0x0FFF // Sin/Lin channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     OTP_CALIB_SETTINGS4_SIN_OFFS_TC_OFFS_TC_POS 0
#define     OTP_CALIB_SETTINGS4_SIN_OFFS_TC_TREF_7_4_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [7:4
#define     OTP_CALIB_SETTINGS4_SIN_OFFS_TC_TREF_7_4_POS 12

#define OTP_CALIB_SETTINGS4_COS_GAIN_TC_REG           0xF7 // Cos channel gain temperature slope
#define     OTP_CALIB_SETTINGS4_COS_GAIN_TC_GAIN_TC_MASK 0x0FFF // Cos channel gain temperature slope. Signed 12 bits, -0.5 to 0.5-2^(-12)
#define     OTP_CALIB_SETTINGS4_COS_GAIN_TC_GAIN_TC_POS 0
#define     OTP_CALIB_SETTINGS4_COS_GAIN_TC_TREF_11_8_MASK 0xF000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bits [3:0
#define     OTP_CALIB_SETTINGS4_COS_GAIN_TC_TREF_11_8_POS 12

#define OTP_CALIB_SETTINGS4_COS_OFFS_TC_REG           0xF8 // Cos channel offset temperature slope
#define     OTP_CALIB_SETTINGS4_COS_OFFS_TC_OFFS_TC_MASK 0x0FFF // Cos channel offset temperature slope. Signed 12 bits, -0.25 to 0.25-2^(-13)
#define     OTP_CALIB_SETTINGS4_COS_OFFS_TC_OFFS_TC_POS 0
#define     OTP_CALIB_SETTINGS4_COS_OFFS_TC_TREF_12_BIT 0x1000 // Reference temperature during calibration (0 to 204 degrees in 25mC steps), bit [12

#define OTP_CALIB_SETTINGS4_OUT_ADJ_REG               0xF9 // Output adjustment settings
#define     OTP_CALIB_SETTINGS4_OUT_ADJ_OFFSET_MASK   0x7FFF // Output angle offset (LSB=360*2^-15 deg)
#define     OTP_CALIB_SETTINGS4_OUT_ADJ_OFFSET_POS    0
#define     OTP_CALIB_SETTINGS4_OUT_ADJ_CW_CCW_SIGN_BIT 0x8000 // CW/CCW output adjustment:
                                                                        // 0: CCW No output adjustment
                                                                        // 1: CW (360 - measure angle)


#define OTP_CALIB_SETTINGS4_OTP_CALIB_CRC_REG         0xFA // CALIB area CRC

#define OTP_USER_AREA_USERDATA0_REG                   0xFB // User Data

#define OTP_USER_AREA_USERDATA1_REG                   0xFC // User Data

#define OTP_USER_AREA_USERDATA2_REG                   0xFD // User Data

#define OTP_USER_AREA_USERDATA3_REG                   0xFE // User Data

#define OTP_USER_AREA_USERDATA4_REG                   0xFF // User Data

