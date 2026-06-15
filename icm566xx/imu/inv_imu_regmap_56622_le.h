/*
 *
 * Copyright (c) [2017] by InvenSense, Inc.
 * * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
#ifndef _VENICE_A1_CHIPTOP_202212210831_6469331EA6B_EXTERNAL_H_
#define _VENICE_A1_CHIPTOP_202212210831_6469331EA6B_EXTERNAL_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @file VENICE_A1_CHIPTOP_202212210831_6469331ea6b.h
 * File exposing the device register map
 */

#include <stdint.h>

/* forward declaration */
struct venice_a1_chiptop_202212210831_6469331ea6b;

/*
 * ----------------------------------------------------------------------------
 * Device Register map
 *
 * Next macros defines address for all venice_a1_chiptop_202212210831_6469331ea6b registers as
 * listed by device * datasheet. Macros name is
 * VENICE_A1_CHIPTOP_202212210831_6469331EA6B_<REGISTER_NAME> with REGISTER_NAME being the name of
 * the corresponding register in datasheet.
 * Note that macro name is VENICE_A1_CHIPTOP_202212210831_6469331EA6B_<REGISTER_NAME>_Bx with x
 * being the bank * number for registers that are in bank 1 and further (suffix is ommitted for *
 * bank 0 registers)
 * ----------------------------------------------------------------------------
 */

/* IPREG_BAR */
#define IPREG_BAR_REG_56 0xa038
typedef struct {
	uint8_t resv_1: 3;
	uint8_t pads_ap_cs_pe_trim_d2a: 1;
	uint8_t pads_ap_cs_pud_trim_d2a: 1;
	uint8_t resv_2: 1;
	uint8_t pads_ap_sclk_pe_trim_d2a: 1;
	uint8_t pads_ap_sclk_pud_trim_d2a: 1;
} ipreg_bar_reg_56_t;

#define IPREG_BAR_REG_57 0xa039
typedef struct {
	uint8_t resv_1: 1;
	uint8_t pads_ap_sdi_pe_trim_d2a: 1;
	uint8_t pads_ap_sdi_pud_trim_d2a: 1;
	uint8_t resv_2: 1;
	uint8_t pads_ap_sdo_pe_trim_d2a: 1;
	uint8_t pads_ap_sdo_pud_trim_d2a: 1;
	uint8_t resv_3: 1;
	uint8_t pads_aux_cs_tp3_tp_pe_trim_d2a: 1;
} ipreg_bar_reg_57_t;

#define IPREG_BAR_REG_58 0xa03a
typedef struct {
	uint8_t pads_aux_cs_tp3_tp_pud_trim_d2a: 1;
	uint8_t resv_1: 1;
	uint8_t pads_aux_sclk_tp2_tp_pe_trim_d2a: 1;
	uint8_t pads_aux_sclk_tp2_tp_pud_trim_d2a: 1;
	uint8_t resv_2: 1;
	uint8_t pads_aux_sdi_tp1_tp_pe_trim_d2a: 1;
	uint8_t pads_aux_sdi_tp1_tp_pud_trim_d2a: 1;
	uint8_t resv_3: 1;
} ipreg_bar_reg_58_t;

#define IPREG_BAR_REG_59 0xa03b
typedef struct {
	uint8_t pads_aux_sdo_pe_trim_d2a: 1;
	uint8_t pads_aux_sdo_pud_trim_d2a: 1;
	uint8_t resv_1: 1;
	uint8_t pads_int1_tp0_tp_pe_trim_d2a: 1;
	uint8_t pads_int1_tp0_tp_pud_trim_d2a: 1;
	uint8_t resv_2: 1;
	uint8_t pads_int2_pe_trim_d2a: 1;
	uint8_t pads_int2_pud_trim_d2a: 1;
} ipreg_bar_reg_59_t;

#define IPREG_BAR_REG_69 0xa045
typedef struct {
	uint8_t pads_od_pud_strength_trim_d2a: 2;
	uint8_t resv_1: 6;
} ipreg_bar_reg_69_t;

#define IPREG_BAR_REG_81 0xa051
typedef struct {
	uint8_t resv_1: 1;
	uint8_t pd_mb_vbgr_test_b_d2a: 1;
	uint8_t pd_minibg_b_d2a: 1;
	uint8_t resv_2: 5;
} ipreg_bar_reg_81_t;

#define IPREG_BAR_REG_83 0xa053
typedef struct {
	uint8_t io_well_bias_comp_sel_d2a: 1;
	uint8_t resv_1: 7;
} ipreg_bar_reg_83_t;

/* IPREG_ANA */
#define IPREG_ANA_REG_5 0xa305
typedef struct {
	uint8_t gyro_drv_xst_mode_d2a: 2;
	uint8_t gyro_drv_yst_mode_d2a: 2;
	uint8_t gyro_drv_zst_mode_d2a: 2;
	uint8_t resv_1: 2;
} ipreg_ana_reg_5_t;

#define IPREG_ANA_REG_6 0xa306
typedef struct {
	uint8_t gyro_drv_xst_pol_d2a: 1;
	uint8_t gyro_drv_yst_pol_d2a: 1;
	uint8_t gyro_drv_zst_pol_d2a: 1;
	uint8_t resv_1: 5;
} ipreg_ana_reg_6_t;

#define IPREG_ANA_REG_8 0xa308
typedef struct {
	uint8_t pd_accel_cp45_st_b_d2a: 1;
	uint8_t pd_accel_st_b_d2a: 1;
	uint8_t resv_1: 6;
} ipreg_ana_reg_8_t;

/* IMEM_ROM */

/* IMEM_SRAM */
#define IMEM_SRAM_REG_0  0x00
#define IMEM_SRAM_REG_1  0x01
#define IMEM_SRAM_REG_2  0x02
#define IMEM_SRAM_REG_3  0x03
#define IMEM_SRAM_REG_4  0x04
#define IMEM_SRAM_REG_5  0x05
#define IMEM_SRAM_REG_6  0x06
#define IMEM_SRAM_REG_7  0x07
#define IMEM_SRAM_REG_8  0x08
#define IMEM_SRAM_REG_9  0x09
#define IMEM_SRAM_REG_10 0x0a
#define IMEM_SRAM_REG_11 0x0b
#define IMEM_SRAM_REG_12 0x0c
#define IMEM_SRAM_REG_13 0x0d
#define IMEM_SRAM_REG_14 0x0e
#define IMEM_SRAM_REG_15 0x0f
#define IMEM_SRAM_REG_16 0x10
#define IMEM_SRAM_REG_17 0x11
#define IMEM_SRAM_REG_18 0x12
#define IMEM_SRAM_REG_19 0x13
#define IMEM_SRAM_REG_20 0x14
#define IMEM_SRAM_REG_21 0x15
#define IMEM_SRAM_REG_22 0x16
#define IMEM_SRAM_REG_23 0x17
#define IMEM_SRAM_REG_24 0x18

/* IMEM_SRAM_APEX */
#define IMEM_SRAM_APEX_REG_168 0xa8
#define IMEM_SRAM_APEX_REG_169 0xa9
#define IMEM_SRAM_APEX_REG_170 0xaa
#define IMEM_SRAM_APEX_REG_171 0xab
#define IMEM_SRAM_APEX_REG_218 0xda
#define IMEM_SRAM_APEX_REG_219 0xdb
#define IMEM_SRAM_APEX_REG_220 0xdc
#define IMEM_SRAM_APEX_REG_221 0xdd
#define IMEM_SRAM_APEX_REG_222 0xde
#define IMEM_SRAM_APEX_REG_223 0xdf

/* IMEM_SRAM_STC */
#define IMEM_SRAM_STC_REG_56  0x38
#define IMEM_SRAM_STC_REG_57  0x39
#define IMEM_SRAM_STC_REG_58  0x3a
#define IMEM_SRAM_STC_REG_59  0x3b
#define IMEM_SRAM_STC_REG_60  0x3c
#define IMEM_SRAM_STC_REG_61  0x3d
#define IMEM_SRAM_STC_REG_62  0x3e
#define IMEM_SRAM_STC_REG_63  0x3f
#define IMEM_SRAM_STC_REG_68  0x44
#define IMEM_SRAM_STC_REG_69  0x45
#define IMEM_SRAM_STC_REG_70  0x46
#define IMEM_SRAM_STC_REG_71  0x47
#define IMEM_SRAM_STC_REG_72  0x48
#define IMEM_SRAM_STC_REG_73  0x49
#define IMEM_SRAM_STC_REG_74  0x4a
#define IMEM_SRAM_STC_REG_75  0x4b
#define IMEM_SRAM_STC_REG_76  0x4c
#define IMEM_SRAM_STC_REG_77  0x4d
#define IMEM_SRAM_STC_REG_78  0x4e
#define IMEM_SRAM_STC_REG_79  0x4f
#define IMEM_SRAM_STC_REG_80  0x50
#define IMEM_SRAM_STC_REG_81  0x51
#define IMEM_SRAM_STC_REG_82  0x52
#define IMEM_SRAM_STC_REG_83  0x53
#define IMEM_SRAM_STC_REG_84  0x54
#define IMEM_SRAM_STC_REG_85  0x55
#define IMEM_SRAM_STC_REG_86  0x56
#define IMEM_SRAM_STC_REG_87  0x57
#define IMEM_SRAM_STC_REG_92  0x5c
#define IMEM_SRAM_STC_REG_93  0x5d
#define IMEM_SRAM_STC_REG_94  0x5e
#define IMEM_SRAM_STC_REG_95  0x5f
#define IMEM_SRAM_STC_REG_96  0x60
#define IMEM_SRAM_STC_REG_97  0x61
#define IMEM_SRAM_STC_REG_98  0x62
#define IMEM_SRAM_STC_REG_99  0x63
#define IMEM_SRAM_STC_REG_100 0x64
#define IMEM_SRAM_STC_REG_101 0x65
#define IMEM_SRAM_STC_REG_102 0x66
#define IMEM_SRAM_STC_REG_103 0x67
#define IMEM_SRAM_STC_REG_104 0x68
#define IMEM_SRAM_STC_REG_105 0x69
#define IMEM_SRAM_STC_REG_106 0x6a
#define IMEM_SRAM_STC_REG_107 0x6b
#define IMEM_SRAM_STC_REG_344 0x158
#define IMEM_SRAM_STC_REG_345 0x159
#define IMEM_SRAM_STC_REG_346 0x15a
#define IMEM_SRAM_STC_REG_347 0x15b
#define IMEM_SRAM_STC_REG_348 0x15c
#define IMEM_SRAM_STC_REG_349 0x15d
#define IMEM_SRAM_STC_REG_350 0x15e
#define IMEM_SRAM_STC_REG_351 0x15f

/* IPREG_FPGA */
#define MSTR_SPI_CTRL 0xa600
typedef struct {
	uint8_t mstr_spi_rst: 1;
	uint8_t sent_to_slave: 1;
	uint8_t mstr_spi_freq: 3;
	uint8_t indirect_access_en: 1;
	uint8_t resv_1: 2;
} mstr_spi_ctrl_t;

#define MSTR_SPI_ADDR 0xa601
typedef struct {
	uint8_t mstr_spi_addr: 8;
} mstr_spi_addr_t;

#define MSTR_SPI_WR_DATA 0xa602
typedef struct {
	uint8_t mstr_spi_wr_data: 8;
} mstr_spi_wr_data_t;

#define MSTR_SPI_RD_DATA 0xa603
typedef struct {
	uint8_t mstr_spi_rd_data: 8;
} mstr_spi_rd_data_t;

#define MSTR_SPI_STATUS 0xa604
typedef struct {
	uint8_t mstr_spi_idle: 1;
	uint8_t mstr_spi_done: 1;
	uint8_t resv_1: 6;
} mstr_spi_status_t;

#define SENSOR_DATA_CTRL 0xa605
typedef struct {
	uint8_t start_sensor: 1;
	uint8_t prologue_en: 1;
	uint8_t acc_sensor_data_en: 1;
	uint8_t gyro_sensor_data_en: 1;
	uint8_t resv_1: 4;
} sensor_data_ctrl_t;

#define FPGA_ID 0xa606
typedef struct {
	uint8_t fpga_id: 8;
} fpga_id_t;

#define FPGA_CONFIG 0xa607
typedef struct {
	uint8_t fpga_root_clk_sel: 1;
	uint8_t resv_1: 7;
} fpga_config_t;

#define REVISION1 0xa608
typedef struct {
	uint8_t fpga_major_revision: 8;
} revision1_t;

#define REVISION2 0xa609
typedef struct {
	uint8_t fpga_minor_revision: 8;
} revision2_t;

#define ACCEL_X_FRE 0xa60a
typedef struct {
	uint8_t accel_x_fre: 8;
} accel_x_fre_t;

#define ACCEL_X_AMP 0xa60b
typedef struct {
	uint8_t accel_x_amp: 8;
} accel_x_amp_t;

#define ACCEL_X_OFS 0xa60c
typedef struct {
	uint8_t accel_x_ofs: 8;
} accel_x_ofs_t;

#define ACCEL_X_PHA 0xa60d
typedef struct {
	uint8_t accel_x_dith_en: 1;
	uint8_t accel_x_pha: 7;
} accel_x_pha_t;

#define ACCEL_X_Y_AEX 0xa60e
typedef struct {
	uint8_t accel_y_aex: 4;
	uint8_t accel_x_aex: 4;
} accel_x_y_aex_t;

#define ACCEL_Y_FRE 0xa60f
typedef struct {
	uint8_t accel_y_fre: 8;
} accel_y_fre_t;

#define ACCEL_Y_AMP 0xa610
typedef struct {
	uint8_t accel_y_amp: 8;
} accel_y_amp_t;

#define ACCEL_Y_OFS 0xa611
typedef struct {
	uint8_t accel_y_ofs: 8;
} accel_y_ofs_t;

#define ACCEL_Y_PHA 0xa612
typedef struct {
	uint8_t accel_y_dith_en: 1;
	uint8_t accel_y_pha: 7;
} accel_y_pha_t;

#define ACCEL_Z_FRE 0xa613
typedef struct {
	uint8_t accel_z_fre: 8;
} accel_z_fre_t;

#define ACCEL_Z_AMP 0xa614
typedef struct {
	uint8_t accel_z_amp: 8;
} accel_z_amp_t;

#define ACCEL_Z_OFS 0xa615
typedef struct {
	uint8_t accel_z_ofs: 8;
} accel_z_ofs_t;

#define ACCEL_Z_PHA 0xa616
typedef struct {
	uint8_t accel_z_dith_en: 1;
	uint8_t accel_z_pha: 7;
} accel_z_pha_t;

#define ACCEL_Z_GYRO_X_AEX 0xa617
typedef struct {
	uint8_t gyro_x_aex: 4;
	uint8_t accel_z_aex: 4;
} accel_z_gyro_x_aex_t;

#define GYRO_X_FRE 0xa618
typedef struct {
	uint8_t gyro_x_fre: 8;
} gyro_x_fre_t;

#define GYRO_X_AMP 0xa619
typedef struct {
	uint8_t gyro_x_amp: 8;
} gyro_x_amp_t;

#define GYRO_X_OFS 0xa61a
typedef struct {
	uint8_t gyro_x_ofs: 8;
} gyro_x_ofs_t;

#define GYRO_X_PHA 0xa61b
typedef struct {
	uint8_t gyro_x_dith_en: 1;
	uint8_t gyro_x_pha: 7;
} gyro_x_pha_t;

#define GYRO_Y_FRE 0xa61c
typedef struct {
	uint8_t gyro_y_fre: 8;
} gyro_y_fre_t;

#define GYRO_Y_AMP 0xa61d
typedef struct {
	uint8_t gyro_y_amp: 8;
} gyro_y_amp_t;

#define GYRO_Y_OFS 0xa61e
typedef struct {
	uint8_t gyro_y_ofs: 8;
} gyro_y_ofs_t;

#define GYRO_Y_PHA 0xa61f
typedef struct {
	uint8_t gyro_y_dith_en: 1;
	uint8_t gyro_y_pha: 7;
} gyro_y_pha_t;

#define GYRO_Y_GYRO_Z_AEX 0xa620
typedef struct {
	uint8_t gyro_z_aex: 4;
	uint8_t gyro_y_aex: 4;
} gyro_y_gyro_z_aex_t;

#define GYRO_Z_FRE 0xa621
typedef struct {
	uint8_t gyro_z_fre: 8;
} gyro_z_fre_t;

#define GYRO_Z_AMP 0xa622
typedef struct {
	uint8_t gyro_z_amp: 8;
} gyro_z_amp_t;

#define GYRO_Z_OFS 0xa623
typedef struct {
	uint8_t gyro_z_ofs: 8;
} gyro_z_ofs_t;

#define GYRO_Z_PHA 0xa624
typedef struct {
	uint8_t gyro_z_dith_en: 1;
	uint8_t gyro_z_pha: 7;
} gyro_z_pha_t;

#define ACCEL_GYRO_CLKDIV 0xa625
typedef struct {
	uint8_t gyro_clk_div: 4;
	uint8_t accel_clk_div: 4;
} accel_gyro_clkdiv_t;

#define FPGA_CTRL 0xa627
typedef struct {
	uint8_t bram_wr_dis: 1;
	uint8_t ram_ovf: 1;
	uint8_t resv_1: 6;
} fpga_ctrl_t;

#define OTP_ERROR_STATUS 0xa628
typedef struct {
	uint8_t diff_read_mismatch_error: 1;
	uint8_t prog_more_than_1_bit_error: 1;
	uint8_t prog_0_bit_error: 1;
	uint8_t cfg_mismatch_error: 1;
	uint8_t resv_1: 4;
} otp_error_status_t;

#define DIFF_READ_ERROR_BITMASK0_0        0xa62a
#define DIFF_READ_ERROR_BITMASK0_1        0xa62b
#define DIFF_READ_ERROR_ADDR0_0           0xa62c
#define DIFF_READ_ERROR_ADDR0_1           0xa62d
#define PROG_MORE_THAN_1BIT_ERROR_DATA0_0 0xa62e
#define PROG_MORE_THAN_1BIT_ERROR_DATA0_1 0xa62f
#define PROG_MORE_THAN_1BIT_ERROR_ADDR0_0 0xa630
#define PROG_MORE_THAN_1BIT_ERROR_ADDR0_1 0xa631
#define PROG_0BIT_ERROR_DATA0_0           0xa632
#define PROG_0BIT_ERROR_DATA0_1           0xa633
#define PROG_0BIT_ERROR_ADDR0_0           0xa634
#define PROG_0BIT_ERROR_ADDR0_1           0xa635
#define FPGA_REG0                         0xa636
typedef struct {
	uint8_t fpga_dbg0: 8;
} fpga_reg0_t;

#define FPGA_REG1 0xa637
typedef struct {
	uint8_t fpga_dbg1: 8;
} fpga_reg1_t;

#define FPGA_REG2 0xa638
typedef struct {
	uint8_t fpga_dbg2: 8;
} fpga_reg2_t;

#define FPGA_REG3 0xa639
typedef struct {
	uint8_t fpga_dbg3: 8;
} fpga_reg3_t;

#define FPGA_REG4 0xa63a
typedef struct {
	uint8_t fpga_dbg4: 8;
} fpga_reg4_t;

#define FPGA_REG5 0xa63b
typedef struct {
	uint8_t fpga_dbg5: 8;
} fpga_reg5_t;

#define FPGA_STAT0 0xa63c
typedef struct {
	uint8_t fpga_dbgstat0: 8;
} fpga_stat0_t;

#define FPGA_STAT1 0xa63d
typedef struct {
	uint8_t fpga_dbgstat1: 8;
} fpga_stat1_t;

/* IPREG_OTP */

/* IPREG_SYS1 */
#define IPREG_SYS1_REG_0   0xa400
#define IPREG_SYS1_REG_2   0xa402
#define IPREG_SYS1_REG_3   0xa403
#define IPREG_SYS1_REG_4   0xa404
#define IPREG_SYS1_REG_5   0xa405
#define IPREG_SYS1_REG_6   0xa406
#define IPREG_SYS1_REG_7   0xa407
#define IPREG_SYS1_REG_8   0xa408
#define IPREG_SYS1_REG_9   0xa409
#define IPREG_SYS1_REG_10  0xa40a
#define IPREG_SYS1_REG_11  0xa40b
#define IPREG_SYS1_REG_12  0xa40c
#define IPREG_SYS1_REG_13  0xa40d
#define IPREG_SYS1_REG_146 0xa492
typedef struct {
	uint8_t gyro_x_tmid_gain: 4;
	uint8_t gyro_x_tmid_off: 4;
} ipreg_sys1_reg_146_t;

#define IPREG_SYS1_REG_148 0xa494
typedef struct {
	uint8_t gyro_y_tmid_gain: 4;
	uint8_t gyro_y_tmid_off: 4;
} ipreg_sys1_reg_148_t;

#define IPREG_SYS1_REG_150 0xa496
typedef struct {
	uint8_t gyro_z_tmid_gain: 4;
	uint8_t gyro_z_tmid_off: 4;
} ipreg_sys1_reg_150_t;

#define IPREG_SYS1_REG_154 0xa49a
typedef struct {
	uint8_t gyro_afsr_enable: 1;
	uint8_t resv_1: 1;
	uint8_t gyro_src_ctrl: 2;
	uint8_t gyro_afsr_shared: 1;
	uint8_t gyro_ois_m6_byp: 1;
	uint8_t resv_2: 2;
} ipreg_sys1_reg_154_t;

#define IPREG_SYS1_REG_155 0xa49b
typedef struct {
	uint8_t gyro_ois_hpf_byp: 1;
	uint8_t gyro_lpf_bypass: 1;
	uint8_t resv_1: 6;
} ipreg_sys1_reg_155_t;

#define IPREG_SYS1_REG_157 0xa49d
typedef struct {
	uint8_t gyro_lp_avg_sel: 4;
	uint8_t gyro_ois_hpfbw_sel: 3;
	uint8_t gyro_notch_bypass: 1;
} ipreg_sys1_reg_157_t;

#define IPREG_SYS1_REG_158 0xa49e
typedef struct {
	uint8_t gyro_ois_lpfbw_sel: 3;
	uint8_t gyro_ois_3rd_ord_sel: 1;
	uint8_t gyro_ui_lpfbw_sel: 3;
	uint8_t gyro_ui_3rd_ord_sel: 1;
} ipreg_sys1_reg_158_t;

/* IPREG_SYS2 */
#define IPREG_SYS2_REG_12  0xa50c
#define IPREG_SYS2_REG_13  0xa50d
#define IPREG_SYS2_REG_14  0xa50e
#define IPREG_SYS2_REG_15  0xa50f
#define IPREG_SYS2_REG_16  0xa510
#define IPREG_SYS2_REG_17  0xa511
#define IPREG_SYS2_REG_18  0xa512
#define IPREG_SYS2_REG_19  0xa513
#define IPREG_SYS2_REG_20  0xa514
#define IPREG_SYS2_REG_21  0xa515
#define IPREG_SYS2_REG_22  0xa516
#define IPREG_SYS2_REG_23  0xa517
#define IPREG_SYS2_REG_105 0xa569
typedef struct {
	uint8_t accel_x_tmid_gain: 4;
	uint8_t accel_x_tmid_off: 4;
} ipreg_sys2_reg_105_t;

#define IPREG_SYS2_REG_106 0xa56a
typedef struct {
	uint8_t accel_y_tmid_gain: 4;
	uint8_t accel_y_tmid_off: 4;
} ipreg_sys2_reg_106_t;

#define IPREG_SYS2_REG_107 0xa56b
typedef struct {
	uint8_t accel_z_tmid_gain: 4;
	uint8_t accel_z_tmid_off: 4;
} ipreg_sys2_reg_107_t;

#define IPREG_SYS2_REG_109 0xa56d
typedef struct {
	uint8_t accel_src_ctrl: 2;
	uint8_t resv_1: 6;
} ipreg_sys2_reg_109_t;

#define IPREG_SYS2_REG_110 0xa56e
typedef struct {
	uint8_t accel_lp_avg_sel: 4;
	uint8_t accel_ois_hpfbw_sel: 3;
	uint8_t resv_1: 1;
} ipreg_sys2_reg_110_t;

#define IPREG_SYS2_REG_111 0xa56f
typedef struct {
	uint8_t accel_ois_lpfbw_sel: 3;
	uint8_t accel_ois_3rd_ord_sel: 1;
	uint8_t resv_1: 4;
} ipreg_sys2_reg_111_t;

#define IPREG_SYS2_REG_112 0xa570
typedef struct {
	uint8_t accel_ui_lpfbw_sel: 3;
	uint8_t accel_ui_3rd_ord_sel: 1;
	uint8_t accel_lpf_bypass: 1;
	uint8_t accel_ois_hpf_byp: 1;
	uint8_t accel_ois_m6_byp: 1;
	uint8_t resv_1: 1;
} ipreg_sys2_reg_112_t;

#define IPREG_SYS2_REG_117 0xa575
typedef struct {
	uint8_t tmp_inj_en_gos: 1;
	uint8_t tmp_lpf_cfg: 3;
	uint8_t tmp_dec_cfg: 3;
	uint8_t resv_1: 1;
} ipreg_sys2_reg_117_t;

/* IPREG_TOP1 */
#define EDMP_PRGRM_IRQ0_0  0xa24f
#define EDMP_PRGRM_IRQ0_1  0xa250
#define EDMP_PRGRM_IRQ1_0  0xa251
#define EDMP_PRGRM_IRQ1_1  0xa252
#define EDMP_PRGRM_IRQ2_0  0xa253
#define EDMP_PRGRM_IRQ2_1  0xa254
#define EDMP_SP_START_ADDR 0xa255
typedef struct {
	uint8_t sp_start_addr: 8;
} edmp_sp_start_addr_t;

#define SMC_CONTROL_0 0xa258
typedef struct {
	uint8_t tmst_en: 1;
	uint8_t tmst_fsync_en: 1;
	uint8_t tmst_force_aux_fine_en: 1;
	uint8_t resv_1: 5;
} smc_control_0_t;

#define SREG_CTRL 0xa260
typedef struct {
	uint8_t sreg_invalid_ind_dis: 1;
	uint8_t sreg_data_endian_sel: 1;
	uint8_t sreg_debug_data_inject_en: 1;
	uint8_t resv_1: 5;
} sreg_ctrl_t;

#define INT_PULSE_MIN_ON_INTF0 0xa262
typedef struct {
	uint8_t int0_tpulse_duration: 3;
	uint8_t resv_1: 5;
} int_pulse_min_on_intf0_t;

#define INT_PULSE_MIN_ON_INTF1 0xa263
typedef struct {
	uint8_t int1_tpulse_duration: 3;
	uint8_t resv_1: 5;
} int_pulse_min_on_intf1_t;

#define INT_PULSE_MIN_OFF_INTF0 0xa264
typedef struct {
	uint8_t int0_tdeassert_disable: 3;
	uint8_t resv_1: 5;
} int_pulse_min_off_intf0_t;

#define INT_PULSE_MIN_OFF_INTF1 0xa265
typedef struct {
	uint8_t int1_tdeassert_disable: 3;
	uint8_t resv_1: 5;
} int_pulse_min_off_intf1_t;

#define STATUS_MASK_PIN_0_7 0xa26a
typedef struct {
	uint8_t int_accel_drdy_pin_0_dis: 1;
	uint8_t int_gyro_drdy_pin_0_dis: 1;
	uint8_t resv_1: 2;
	uint8_t int_wom_drdy_pin_0_dis: 1;
	uint8_t int_on_demand_pin_0_dis: 1;
	uint8_t resv_2: 2;
} status_mask_pin_0_7_t;

#define STATUS_MASK_PIN_8_15 0xa26b
typedef struct {
	uint8_t int_accel_drdy_pin_1_dis: 1;
	uint8_t int_gyro_drdy_pin_1_dis: 1;
	uint8_t resv_1: 2;
	uint8_t int_wom_drdy_pin_1_dis: 1;
	uint8_t int_on_demand_pin_1_dis: 1;
	uint8_t resv_2: 2;
} status_mask_pin_8_15_t;

#define STATUS_MASK_PIN_16_23 0xa26c
typedef struct {
	uint8_t int_accel_drdy_pin_2_dis: 1;
	uint8_t int_gyro_drdy_pin_2_dis: 1;
	uint8_t resv_1: 2;
	uint8_t int_wom_drdy_pin_2_dis: 1;
	uint8_t int_on_demand_pin_2_dis: 1;
	uint8_t resv_2: 2;
} status_mask_pin_16_23_t;

#define ACCEL_WOM_X_THR 0xa277
typedef struct {
	uint8_t wom_x_th: 8;
} accel_wom_x_thr_t;

#define ACCEL_WOM_Y_THR 0xa278
typedef struct {
	uint8_t wom_y_th: 8;
} accel_wom_y_thr_t;

#define ACCEL_WOM_Z_THR 0xa279
typedef struct {
	uint8_t wom_z_th: 8;
} accel_wom_z_thr_t;

#define IOC_PADS_CONFIG0 0xa27d
typedef struct {
	uint8_t resv_1: 5;
	uint8_t fsync_polarity: 1;
	uint8_t resv_2: 2;
} ioc_pads_config0_t;

#define IREG_OTP_CFG 0xa282
typedef struct {
	uint8_t otp_copy_mode: 2;
	uint8_t resv_1: 6;
} ireg_otp_cfg_t;

#define SELFTEST 0xa289
typedef struct {
	uint8_t en_ax_st: 1;
	uint8_t en_ay_st: 1;
	uint8_t en_az_st: 1;
	uint8_t en_gx_st: 1;
	uint8_t en_gy_st: 1;
	uint8_t en_gz_st: 1;
	uint8_t resv_1: 2;
} selftest_t;

#define IPREG_MISC 0xa290
typedef struct {
	uint8_t resv_1: 1;
	uint8_t edmp_idle: 1;
	uint8_t resv_2: 6;
} ipreg_misc_t;

#define SW_RCOSC1_TRIM 0xa2a1
typedef struct {
	uint8_t sw_rcosc1_trim: 8;
} sw_rcosc1_trim_t;

#define SW_PLL1_TRIM 0xa2a2
typedef struct {
	uint8_t sw_pll1_trim: 8;
} sw_pll1_trim_t;

#define FIFO_SRAM_SLEEP 0xa2a7
typedef struct {
	uint8_t fifo_gsleep_shared_sram: 1;
	uint8_t resv_1: 7;
} fifo_sram_sleep_t;

/* DREG_BANK1 */
#define ACCEL_DATA_X_0    0x00
#define ACCEL_DATA_X_1    0x01
#define ACCEL_DATA_Y_0    0x02
#define ACCEL_DATA_Y_1    0x03
#define ACCEL_DATA_Z_0    0x04
#define ACCEL_DATA_Z_1    0x05
#define GYRO_DATA_X_0     0x06
#define GYRO_DATA_X_1     0x07
#define GYRO_DATA_Y_0     0x08
#define GYRO_DATA_Y_1     0x09
#define GYRO_DATA_Z_0     0x0a
#define GYRO_DATA_Z_1     0x0b
#define TEMP_DATA_0       0x0c
#define TEMP_DATA_1       0x0d
#define TMST_FSYNC_DATA_0 0x0e
#define TMST_FSYNC_DATA_1 0x0f

#define PWR_MGMT0 0x14
typedef struct {
	uint8_t accel_mode: 2;
	uint8_t gyro_mode: 2;
	uint8_t resv_1: 1;
	uint8_t accel_lp_clk_sel: 1;
	uint8_t resv_2: 2;
} pwr_mgmt0_t;

#define FIFO_COUNT_0 0x16
#define FIFO_COUNT_1 0x17
#define FIFO_DATA    0x18
typedef struct {
	uint8_t fifo_rdata: 8;
} fifo_data_t;

#define INT1_CONFIG0 0x1a
typedef struct {
	uint8_t int1_status_en_fifo_full: 1;
	uint8_t int1_status_en_fifo_ths: 1;
	uint8_t int1_status_en_drdy: 1;
	uint8_t int1_status_en_aux1_drdy: 1;
	uint8_t int1_status_en_ap_fsync: 1;
	uint8_t int1_status_en_ap_agc_rdy: 1;
	uint8_t int1_status_en_aux1_agc_rdy: 1;
	uint8_t int1_status_en_reset_done: 1;
} int1_config0_t;

#define INT1_CONFIG1 0x1b
typedef struct {
	uint8_t int1_status_en_pll_rdy: 1;
	uint8_t int1_status_en_wom_x: 1;
	uint8_t int1_status_en_wom_y: 1;
	uint8_t int1_status_en_wom_z: 1;
	uint8_t int1_status_en_i3c_protocol_err: 1;
	uint8_t resv_1: 1;
	uint8_t int1_status_en_apex_event: 1;
	uint8_t resv_2: 1;
} int1_config1_t;

#define INT1_CONFIG2 0x1c
typedef struct {
	uint8_t int1_polarity: 1;
	uint8_t int1_mode: 1;
	uint8_t int1_drive: 1;
	uint8_t resv_1: 5;
} int1_config2_t;

#define INT1_STATUS0 0x1d
typedef struct {
	uint8_t int1_status_fifo_full: 1;
	uint8_t int1_status_fifo_ths: 1;
	uint8_t int1_status_drdy: 1;
	uint8_t int1_status_aux1_drdy: 1;
	uint8_t int1_status_ap_fsync: 1;
	uint8_t int1_status_ap_agc_rdy: 1;
	uint8_t int1_status_aux1_agc_rdy: 1;
	uint8_t int1_status_reset_done: 1;
} int1_status0_t;

#define INT1_STATUS1 0x1e
typedef struct {
	uint8_t int1_status_pll_rdy: 1;
	uint8_t int1_status_wom_x: 1;
	uint8_t int1_status_wom_y: 1;
	uint8_t int1_status_wom_z: 1;
	uint8_t int1_status_i3c_protocol_err: 1;
	uint8_t resv_1: 1;
	uint8_t int1_status_apex_event: 1;
	uint8_t resv_2: 1;
} int1_status1_t;

#define ACCEL_CONFIG0 0x1f
typedef struct {
	uint8_t accel_odr: 4;
	uint8_t ap_accel_fs_sel: 3;
	uint8_t resv_1: 1;
} accel_config0_t;

#define GYRO_CONFIG0 0x20
typedef struct {
	uint8_t gyro_odr: 4;
	uint8_t ap_gyro_fs_sel: 4;
} gyro_config0_t;

#define FIFO_CONFIG0 0x21
typedef struct {
	uint8_t fifo_depth: 6;
	uint8_t fifo_mode: 2;
} fifo_config0_t;

#define FIFO_CONFIG1_0 0x22
#define FIFO_CONFIG1_1 0x23
#define FIFO_CONFIG2   0x24
typedef struct {
	uint8_t resv_1: 3;
	uint8_t fifo_wr_wm_gt_th: 1;
	uint8_t fifo_int_ovfl: 1;
	uint8_t resv_2: 2;
	uint8_t fifo_flush: 1;
} fifo_config2_t;

#define FIFO_CONFIG3 0x25
typedef struct {
	uint8_t fifo_if_en: 1;
	uint8_t fifo_accel_en: 1;
	uint8_t fifo_gyro_en: 1;
	uint8_t fifo_hires_en: 1;
	uint8_t resv_1: 4;
} fifo_config3_t;

#define FIFO_CONFIG4 0x26
typedef struct {
	uint8_t fifo_tmst_fsync_en: 1;
	uint8_t fifo_comp_en: 1;
	uint8_t fifo_comp_nc_flow_cfg: 3;
	uint8_t resv_1: 3;
} fifo_config4_t;

#define TMST_WOM_CONFIG 0x27
typedef struct {
	uint8_t wom_int_dur: 2;
	uint8_t wom_int_mode: 1;
	uint8_t wom_mode: 1;
	uint8_t wom_en: 1;
	uint8_t tmst_resol: 1;
	uint8_t tmst_delta_en: 1;
	uint8_t resv_1: 1;
} tmst_wom_config_t;

#define FSYNC_CONFIG0 0x28
typedef struct {
	uint8_t ap_fsync_sel: 3;
	uint8_t ap_fsync_flag_clear_sel: 1;
	uint8_t resv_1: 4;
} fsync_config0_t;

#define FSYNC_CONFIG1 0x29
typedef struct {
	uint8_t aux1_fsync_sel: 3;
	uint8_t aux1_fsync_flag_clear_sel: 1;
	uint8_t resv_1: 4;
} fsync_config1_t;

#define OTP_HEATER_RTC_CONFIG 0x2a
typedef struct {
	uint8_t resv_1: 5;
	uint8_t rtc_mode: 1;
	uint8_t rtc_align: 1;
	uint8_t resv_2: 1;
} otp_heater_rtc_config_t;

#define DMP_EXT_SEN_ODR_CFG 0x2b
typedef struct {
	uint8_t apex_odr: 3;
	uint8_t resv_1: 5;
} dmp_ext_sen_odr_cfg_t;

#define ODR_DECIMATE_CONFIG 0x2c
typedef struct {
	uint8_t accel_fifo_odr_dec: 4;
	uint8_t gyro_fifo_odr_dec: 4;
} odr_decimate_config_t;

#define EDMP_APEX_EN0 0x2d
typedef struct {
	uint8_t reserved1: 5;
	uint8_t ff_en: 1;
	uint8_t reserved2: 2;
} edmp_apex_en0_t;

#define EDMP_APEX_EN1 0x2e
typedef struct {
	uint8_t reserved1: 1;
	uint8_t init_en: 1;
	uint8_t power_save_en: 1;
	uint8_t reserved2: 3;
	uint8_t edmp_enable: 1;
	uint8_t reserved3: 1;
} edmp_apex_en1_t;

#define INTF_CONFIG0 0x30
typedef struct {
	uint8_t ap_spi_mode: 1;
	uint8_t ap_spi_34_mode: 1;
	uint8_t resv_1: 6;
} intf_config0_t;

#define INTF_CONFIG1_OVRD 0x31
typedef struct {
	uint8_t ap_spi_mode_ovrd_val: 1;
	uint8_t ap_spi_mode_ovrd: 1;
	uint8_t ap_spi_34_mode_ovrd_val: 1;
	uint8_t ap_spi_34_mode_ovrd: 1;
	uint8_t resv_1: 4;
} intf_config1_ovrd_t;

#define INTF_AUX_CONFIG 0x32
typedef struct {
	uint8_t aux1_spi_mode: 1;
	uint8_t aux1_spi_34_mode: 1;
	uint8_t resv_1: 6;
} intf_aux_config_t;

#define IOC_PAD_SCENARIO_OVRD 0x35
typedef struct {
	uint8_t pads_int2_cfg_ovrd_val: 2;
	uint8_t pads_int2_cfg_ovrd: 1;
	uint8_t resv_1: 5;
} ioc_pad_scenario_ovrd_t;

#define DRIVE_CONFIG0 0x36
typedef struct {
	uint8_t pads_spi_slew: 3;
	uint8_t pads_i2c_slew: 3;
	uint8_t resv_1: 2;
} drive_config0_t;

#define DRIVE_CONFIG1 0x37
typedef struct {
	uint8_t pads_i3c_sdr_slew: 3;
	uint8_t pads_i3c_ddr_slew: 3;
	uint8_t resv_1: 2;
} drive_config1_t;

#define DRIVE_CONFIG2 0x38
typedef struct {
	uint8_t pads_slew: 3;
	uint8_t resv_1: 5;
} drive_config2_t;

#define INT_APEX_CONFIG0 0x3d
typedef struct {
	uint8_t int_status_mask_pin_tap_detect: 1;
	uint8_t int_status_mask_pin_high_g_det: 1;
	uint8_t int_status_mask_pin_low_g_det: 1;
	uint8_t int_status_mask_pin_tilt_det: 1;
	uint8_t int_status_mask_pin_step_cnt_ovfl: 1;
	uint8_t int_status_mask_pin_step_det: 1;
	uint8_t int_status_mask_pin_ff_det: 1;
	uint8_t int_status_mask_pin_r2w_wake_det: 1;
} int_apex_config0_t;

#define INT_APEX_CONFIG1 0x3e
typedef struct {
	uint8_t int_status_mask_pin_r2w_sleep_det: 1;
	uint8_t int_status_mask_pin_smd_det: 1;
	uint8_t int_status_mask_pin_selftest_done: 1;
	uint8_t reserved: 1;
	uint8_t int_status_mask_pin_sa_done: 1;
	uint8_t int_status_mask_pin_shake_det: 1;
	uint8_t int_status_mask_pin_b2s_det: 1;
	uint8_t int_status_mask_pin_revb2s_det: 1;
} int_apex_config1_t;

#define INT_APEX_CONFIG2 0x3f
typedef struct {
	uint8_t int_status_mask_pin_nomotion_det: 1;
	uint8_t int_status_mask_pin_motion_det: 1;
	uint8_t int_status_mask_pin_noflat_det: 1;
	uint8_t int_status_mask_pin_flat_det: 1;
	uint8_t resv_1: 4;
} int_apex_config2_t;

#define INT_APEX_STATUS0 0x40
typedef struct {
	uint8_t int_status_tap_detect: 1;
	uint8_t int_status_high_g_det: 1;
	uint8_t int_status_low_g_det: 1;
	uint8_t int_status_tilt_det: 1;
	uint8_t int_status_step_cnt_ovfl: 1;
	uint8_t int_status_step_det: 1;
	uint8_t int_status_ff_det: 1;
	uint8_t int_status_r2w_wake_det: 1;
} int_apex_status0_t;

#define INT_APEX_STATUS1 0x41
typedef struct {
	uint8_t int_status_r2w_sleep_det: 1;
	uint8_t int_status_smd_det: 1;
	uint8_t int_status_selftest_done: 1;
	uint8_t reserved: 1;
	uint8_t int_status_sa_done: 1;
	uint8_t int_status_shake_det: 1;
	uint8_t int_status_b2s_det: 1;
	uint8_t int_status_revb2s_det: 1;
} int_apex_status1_t;

#define INT_APEX_STATUS2 0x42
typedef struct {
	uint8_t int_status_nomotion_det: 1;
	uint8_t int_status_motion_det: 1;
	uint8_t int_status_noflat_det: 1;
	uint8_t int_status_flat_det: 1;
	uint8_t resv_1: 4;
} int_apex_status2_t;

#define INTF_CONFIG_OVRD_AUX1 0x46
typedef struct {
	uint8_t aux1_spi_mode_ovrd_val: 1;
	uint8_t aux1_spi_mode_ovrd: 1;
	uint8_t aux1_spi_34_mode_ovrd_val: 1;
	uint8_t aux1_spi_34_mode_ovrd: 1;
	uint8_t resv_1: 4;
} intf_config_ovrd_aux1_t;

#define ACCEL_DATA_X_AUX1_0    0x48
#define ACCEL_DATA_X_AUX1_1    0x49
#define ACCEL_DATA_Y_AUX1_0    0x4a
#define ACCEL_DATA_Y_AUX1_1    0x4b
#define ACCEL_DATA_Z_AUX1_0    0x4c
#define ACCEL_DATA_Z_AUX1_1    0x4d
#define GYRO_DATA_X_AUX1_0     0x4e
#define GYRO_DATA_X_AUX1_1     0x4f
#define GYRO_DATA_Y_AUX1_0     0x50
#define GYRO_DATA_Y_AUX1_1     0x51
#define GYRO_DATA_Z_AUX1_0     0x52
#define GYRO_DATA_Z_AUX1_1     0x53
#define TEMP_DATA_AUX1_0       0x54
#define TEMP_DATA_AUX1_1       0x55
#define TMST_FSYNC_DATA_AUX1_0 0x56
#define TMST_FSYNC_DATA_AUX1_1 0x57
#define PWR_MGMT_AUX1          0x58
typedef struct {
	uint8_t accel_aux1_en: 1;
	uint8_t gyro_aux1_en: 1;
	uint8_t resv_1: 6;
} pwr_mgmt_aux1_t;

#define FS_SEL_AUX1 0x59
typedef struct {
	uint8_t aux1_accel_fs_sel: 3;
	uint8_t aux1_gyro_fs_sel: 4;
	uint8_t resv_1: 1;
} fs_sel_aux1_t;

#define INT2_CONFIG0 0x5a
typedef struct {
	uint8_t int2_status_en_fifo_full: 1;
	uint8_t int2_status_en_fifo_ths: 1;
	uint8_t int2_status_en_drdy: 1;
	uint8_t int2_status_en_aux1_drdy: 1;
	uint8_t int2_status_en_ap_fsync: 1;
	uint8_t int2_status_en_ap_agc_rdy: 1;
	uint8_t int2_status_en_aux1_agc_rdy: 1;
	uint8_t int2_status_en_reset_done: 1;
} int2_config0_t;

#define INT2_CONFIG1 0x5b
typedef struct {
	uint8_t int2_status_en_pll_rdy: 1;
	uint8_t int2_status_en_wom_x: 1;
	uint8_t int2_status_en_wom_y: 1;
	uint8_t int2_status_en_wom_z: 1;
	uint8_t int2_status_en_i3c_protocol_err: 1;
	uint8_t resv_1: 1;
	uint8_t int2_status_en_apex_event: 1;
	uint8_t resv_2: 1;
} int2_config1_t;

#define INT2_CONFIG2 0x5c
typedef struct {
	uint8_t int2_polarity: 1;
	uint8_t int2_mode: 1;
	uint8_t int2_drive: 1;
	uint8_t resv_1: 5;
} int2_config2_t;

#define INT2_STATUS0 0x5d
typedef struct {
	uint8_t int2_status_fifo_full: 1;
	uint8_t int2_status_fifo_ths: 1;
	uint8_t int2_status_drdy: 1;
	uint8_t int2_status_aux1_drdy: 1;
	uint8_t int2_status_ap_fsync: 1;
	uint8_t int2_status_ap_agc_rdy: 1;
	uint8_t int2_status_aux1_agc_rdy: 1;
	uint8_t int2_status_reset_done: 1;
} int2_status0_t;

#define INT2_STATUS1 0x5e
typedef struct {
	uint8_t int2_status_pll_rdy: 1;
	uint8_t int2_status_wom_x: 1;
	uint8_t int2_status_wom_y: 1;
	uint8_t int2_status_wom_z: 1;
	uint8_t int2_status_i3c_protocol_err: 1;
	uint8_t resv_1: 1;
	uint8_t int2_status_apex_event: 1;
	uint8_t resv_2: 1;
} int2_status1_t;

#define EDMP_GPR_0    0x6c
#define EDMP_GPR_1    0x6d
#define EDMP_GPR_2    0x6e
#define EDMP_GPR_3    0x6f
#define EDMP_GPR_CTRL 0x70
typedef struct {
	uint8_t edmp_gpr_ctrl: 4;
	uint8_t resv_1: 4;
} edmp_gpr_ctrl_t;

#define WHO_AM_I 0x72
typedef struct {
	uint8_t who_am_i: 8;
} who_am_i_t;

#define REG_HOST_MSG 0x73
typedef struct {
	uint8_t testopenable: 1;
	uint8_t resv_1: 3;
	uint8_t sa_en: 1;
	uint8_t edmp_on_demand_en: 1;
	uint8_t resv_2: 2;
} reg_host_msg_t;

#define IREG_ADDR_15_8 0x7c
typedef struct {
	uint8_t ireg_addr_15_8: 8;
} ireg_addr_15_8_t;

#define IREG_ADDR_7_0 0x7d
typedef struct {
	uint8_t ireg_addr_7_0: 8;
} ireg_addr_7_0_t;

#define IREG_DATA 0x7e
typedef struct {
	uint8_t ireg_data: 8;
} ireg_data_t;

#define REG_MISC2 0x7f
typedef struct {
	uint8_t ireg_done: 1;
	uint8_t soft_rst: 1;
	uint8_t resv_1: 6;
} reg_misc2_t;

#ifdef __cplusplus
}
#endif

#endif /*#ifndef _VENICE_A1_CHIPTOP_202212210831_6469331EA6B_EXTERNAL_H_*/
