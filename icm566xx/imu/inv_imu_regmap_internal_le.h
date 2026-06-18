/*
 * Copyright (c) 2017 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
 */
#ifndef _VENICE_A1_CHIPTOP_202212210831_6469331EA6B_INTERNAL_H_
#define _VENICE_A1_CHIPTOP_202212210831_6469331EA6B_INTERNAL_H_

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
#define IPREG_BAR_REG_0 0xa000
typedef struct {
	uint8_t edosc_freq_trim_d2d: 8;
} ipreg_bar_reg_0_internal_t;

#define IPREG_BAR_REG_1 0xa001
typedef struct {
	uint8_t gyro_bias_bg_trim_d2a: 8;
} ipreg_bar_reg_1_internal_t;

#define IPREG_BAR_REG_2 0xa002
typedef struct {
	uint8_t gyro_bias_areg_cp25v_trim_d2a: 5;
	uint8_t gyro_bias_spare1_trim_d2a: 3;
} ipreg_bar_reg_2_internal_t;

#define IPREG_BAR_REG_3 0xa003
typedef struct {
	uint8_t gyro_bias_areg_trim_d2a: 5;
	uint8_t gyro_bias_spare2_trim_d2a: 3;
} ipreg_bar_reg_3_internal_t;

#define IPREG_BAR_REG_4 0xa004
typedef struct {
	uint8_t gyro_bias_curv_corr_trim_d2a: 4;
	uint8_t gyro_bias_spare3_trim_d2a: 3;
	uint8_t gyro_bias_dis_curv_corr_trim_d2a: 1;
} ipreg_bar_reg_4_internal_t;

#define IPREG_BAR_REG_5 0xa005
typedef struct {
	uint8_t gyro_bias_psrr_trim_d2a: 1;
	uint8_t resv_1: 7;
} ipreg_bar_reg_5_internal_t;

#define IPREG_BAR_REG_6 0xa006
typedef struct {
	uint8_t spare3_trim_d2a: 8;
} ipreg_bar_reg_6_internal_t;

#define IPREG_BAR_REG_7 0xa007
typedef struct {
	uint8_t wuosc_freq_trim_d2d: 8;
} ipreg_bar_reg_7_internal_t;

#define IPREG_BAR_REG_8 0xa008
typedef struct {
	uint8_t gz_sc2v_cc_hfs_trim_d2a: 7;
	uint8_t gz_sc2v_lco_trim_d2a: 1;
} ipreg_bar_reg_8_internal_t;

#define IPREG_BAR_REG_9 0xa009
typedef struct {
	uint8_t gz_sc2v_cc_lfs_trim_d2a: 7;
	uint8_t gz_sc2v_spare_trim_d2a: 1;
} ipreg_bar_reg_9_internal_t;

#define IPREG_BAR_REG_10 0xa00a
typedef struct {
	uint8_t gz_sc2v_ft_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_10_internal_t;

#define IPREG_BAR_REG_11 0xa00b
typedef struct {
	uint8_t gz_sc2v_gain_hfs_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_11_internal_t;

#define IPREG_BAR_REG_12 0xa00c
typedef struct {
	uint8_t gz_sc2v_gain_lfs_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_12_internal_t;

#define IPREG_BAR_REG_13 0xa00d
typedef struct {
	uint8_t gz_sc2v_soc_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_13_internal_t;

#define IPREG_BAR_REG_14 0xa00e
typedef struct {
	uint8_t gz_sc2v_off_cm_trim_d2a: 4;
	uint8_t gz_sc2v_off_diff_trim_d2a: 4;
} ipreg_bar_reg_14_internal_t;

#define IPREG_BAR_REG_15 0xa00f
typedef struct {
	uint8_t gz_sc2v_ptat2_hfs_trim_d2a: 4;
	uint8_t gz_sc2v_ptat2_lfs_trim_d2a: 4;
} ipreg_bar_reg_15_internal_t;

#define IPREG_BAR_REG_16 0xa010
typedef struct {
	uint8_t gz_sc2v_vbgr2_hfs_trim_d2a: 4;
	uint8_t gz_sc2v_vbgr2_lfs_trim_d2a: 4;
} ipreg_bar_reg_16_internal_t;

#define IPREG_BAR_REG_17 0xa011
typedef struct {
	uint8_t gz_sc2v_ptat1_trim_d2a: 3;
	uint8_t gz_sc2v_vbgr1_trim_d2a: 3;
	uint8_t gz_sc2v_ostage_trim_d2a: 2;
} ipreg_bar_reg_17_internal_t;

#define IPREG_BAR_REG_18 0xa012
typedef struct {
	uint8_t gz_dmd_offset_hfs_trim_d2a: 5;
	uint8_t gz_dmd_mixer_lo_inv_trim_d2a: 1;
	uint8_t resv_1: 2;
} ipreg_bar_reg_18_internal_t;

#define IPREG_BAR_REG_19 0xa013
typedef struct {
	uint8_t gz_dmd_offset_lfs_trim_d2a: 5;
	uint8_t resv_1: 3;
} ipreg_bar_reg_19_internal_t;

#define IPREG_BAR_REG_20 0xa014
typedef struct {
	uint8_t gz_dmd_spare_trim_d2a: 4;
	uint8_t gz_integ_offset_trim_d2a: 4;
} ipreg_bar_reg_20_internal_t;

#define IPREG_BAR_REG_21 0xa015
typedef struct {
	uint8_t gyro_bias_spare_d2a: 8;
} ipreg_bar_reg_21_internal_t;

#define IPREG_BAR_REG_22 0xa016
typedef struct {
	uint8_t pd_gyro_bias_adcvcmbuffer_b_d2a: 1;
	uint8_t pd_gyro_bias_areg_b_d2a: 1;
	uint8_t pd_gyro_bias_areg_cp25v_b_d2a: 1;
	uint8_t pd_gyro_bias_bg_b_d2a: 1;
	uint8_t pd_gyro_bias_ibiasgen_b_d2a: 1;
	uint8_t pd_gyro_bias_refbuffer_b_d2a: 1;
	uint8_t pd_gyro_bias_vcmbuffer_b_d2a: 1;
	uint8_t resv_1: 1;
} ipreg_bar_reg_22_internal_t;

#define IPREG_BAR_REG_23 0xa017
typedef struct {
	uint8_t spare3_d2a: 8;
} ipreg_bar_reg_23_internal_t;

#define IPREG_BAR_REG_24 0xa018
typedef struct {
	uint8_t gz_sc2v_cfg_spare_d2a: 4;
	uint8_t gz_sc2v_en_se_d2a: 2;
	uint8_t gz_sc2v_dc2v_ph_meas_d2a: 1;
	uint8_t resv_1: 1;
} ipreg_bar_reg_24_internal_t;

#define IPREG_BAR_REG_25 0xa019
typedef struct {
	uint8_t gz_dmd_spare_d2a: 4;
	uint8_t gz_dmd_test_d2a: 4;
} ipreg_bar_reg_25_internal_t;

#define IPREG_BAR_REG_26 0xa01a
typedef struct {
	uint8_t pd_gz_adc_b_d2a: 1;
	uint8_t resv_1: 7;
} ipreg_bar_reg_26_internal_t;

#define IPREG_BAR_REG_27 0xa01b
typedef struct {
	uint8_t accel_bias_spare_trim_d2a: 8;
} ipreg_bar_reg_27_internal_t;

#define IPREG_BAR_REG_28 0xa01c
typedef struct {
	uint8_t accel_areg_cp_trim_d2a: 6;
	uint8_t accel_drive_static_mode_trim_d2a: 1;
	uint8_t accel_vref0p7_unbuffered_trim_d2a: 1;
} ipreg_bar_reg_28_internal_t;

#define IPREG_BAR_REG_29 0xa01d
typedef struct {
	uint8_t accel_areg_trim_d2a: 6;
	uint8_t az_vrefshield_out2avdd_bs_trim_d2a: 1;
	uint8_t az_vrefshield_out2avdd_z_trim_d2a: 1;
} ipreg_bar_reg_29_internal_t;

#define IPREG_BAR_REG_30 0xa01e
typedef struct {
	uint8_t az_vrefshield_bs_trim_d2a: 5;
	uint8_t az_vrefshield_out2hiz_bs_trim_d2a: 1;
	uint8_t az_vrefshield_out2hiz_z_trim_d2a: 1;
	uint8_t bs_short2vref_trim_d2a: 1;
} ipreg_bar_reg_30_internal_t;

#define IPREG_BAR_REG_31 0xa01f
typedef struct {
	uint8_t az_vrefshield_z_trim_d2a: 5;
	uint8_t en_accel_drive_chop_trim_d2a: 1;
	uint8_t en_accel_drivebuffer_vd1p4v_trim_d2a: 1;
	uint8_t pd_axy_vrefshield_b_trim_d2a: 1;
} ipreg_bar_reg_31_internal_t;

#define IPREG_BAR_REG_32 0xa020
typedef struct {
	uint8_t pd_az_negcp_b_trim_d2a: 1;
	uint8_t pd_az_vrefshield_b_trim_d2a: 1;
	uint8_t resv_1: 6;
} ipreg_bar_reg_32_internal_t;

#define IPREG_BAR_REG_33 0xa021
typedef struct {
	uint8_t accel_spare_trim_d2a: 8;
} ipreg_bar_reg_33_internal_t;

#define IPREG_BAR_REG_34 0xa022
typedef struct {
	uint8_t ax_cc_trim_d2d: 6;
	uint8_t accel_testbuffer_guardband_trim_d2d: 2;
} ipreg_bar_reg_34_internal_t;

#define IPREG_BAR_REG_35 0xa023
typedef struct {
	uint8_t ax_cfb_trim_d2d: 6;
	uint8_t accel_integ_chop_trim_d2a: 1;
	uint8_t ax_dmd_inv_trim_d2d: 1;
} ipreg_bar_reg_35_internal_t;

#define IPREG_BAR_REG_36 0xa024
typedef struct {
	uint8_t ax_ft_trim_d2d: 6;
	uint8_t ax_drv_phase_trim_d2a: 1;
	uint8_t axy_2x_speed_trim_d2d: 1;
} ipreg_bar_reg_36_internal_t;

#define IPREG_BAR_REG_37 0xa025
typedef struct {
	uint8_t ay_cc_trim_d2d: 6;
	uint8_t ay_dmd_inv_trim_d2d: 1;
	uint8_t ay_drv_phase_trim_d2a: 1;
} ipreg_bar_reg_37_internal_t;

#define IPREG_BAR_REG_38 0xa026
typedef struct {
	uint8_t ay_cfb_trim_d2d: 6;
	uint8_t az_2x_speed_trim_d2d: 1;
	uint8_t az_dmd_inv_trim_d2d: 1;
} ipreg_bar_reg_38_internal_t;

#define IPREG_BAR_REG_39 0xa027
typedef struct {
	uint8_t ay_ft_trim_d2d: 6;
	uint8_t az_drv_phase_trim_d2a: 1;
	uint8_t az_mems_gain_trim_d2a: 1;
} ipreg_bar_reg_39_internal_t;

#define IPREG_BAR_REG_40 0xa028
typedef struct {
	uint8_t az_cc_trim_d2d: 6;
	uint8_t resv_1: 2;
} ipreg_bar_reg_40_internal_t;

#define IPREG_BAR_REG_41 0xa029
typedef struct {
	uint8_t az_cfb_trim_d2d: 6;
	uint8_t resv_1: 2;
} ipreg_bar_reg_41_internal_t;

#define IPREG_BAR_REG_42 0xa02a
typedef struct {
	uint8_t az_ft_trim_d2d: 6;
	uint8_t resv_1: 2;
} ipreg_bar_reg_42_internal_t;

#define IPREG_BAR_REG_43 0xa02b
typedef struct {
	uint8_t accel_vbg_trim_d2a: 5;
	uint8_t resv_1: 3;
} ipreg_bar_reg_43_internal_t;

#define IPREG_BAR_REG_44 0xa02c
typedef struct {
	uint8_t accel_adc_rc_trim_d2a: 4;
	uint8_t accel_adc_ref_trim_d2a: 4;
} ipreg_bar_reg_44_internal_t;

#define IPREG_BAR_REG_45 0xa02d
typedef struct {
	uint8_t accel_c2v_ctrl_trim_d2a: 4;
	uint8_t accel_integ_rc_trim_d2a: 4;
} ipreg_bar_reg_45_internal_t;

#define IPREG_BAR_REG_46 0xa02e
typedef struct {
	uint8_t accel_vbgr_trim_d2a: 4;
	uint8_t ax_cd_trim_d2a: 4;
} ipreg_bar_reg_46_internal_t;

#define IPREG_BAR_REG_47 0xa02f
typedef struct {
	uint8_t ax_cpmgnd_trim_d2a: 4;
	uint8_t ax_cpmsh_trim_d2a: 4;
} ipreg_bar_reg_47_internal_t;

#define IPREG_BAR_REG_48 0xa030
typedef struct {
	uint8_t ay_cd_trim_d2a: 4;
	uint8_t ay_cpmgnd_trim_d2a: 4;
} ipreg_bar_reg_48_internal_t;

#define IPREG_BAR_REG_49 0xa031
typedef struct {
	uint8_t ay_cpmsh_trim_d2a: 4;
	uint8_t resv_1: 4;
} ipreg_bar_reg_49_internal_t;

#define IPREG_BAR_REG_50 0xa032
typedef struct {
	uint8_t mbcore_spare_trim_d2a: 8;
} ipreg_bar_reg_50_internal_t;

#define IPREG_BAR_REG_51 0xa033
typedef struct {
	uint8_t minibg_ctat_trim_d2a: 5;
	uint8_t mb_startup_dis_trim_d2a: 1;
	uint8_t resv_1: 2;
} ipreg_bar_reg_51_internal_t;

#define IPREG_BAR_REG_52 0xa034
typedef struct {
	uint8_t minibg_vbg_trim_d2a: 5;
	uint8_t resv_1: 3;
} ipreg_bar_reg_52_internal_t;

#define IPREG_BAR_REG_53 0xa035
typedef struct {
	uint8_t mbcore_vbgr_trim_d2a: 4;
	uint8_t minibg_ptat_trim_d2a: 4;
} ipreg_bar_reg_53_internal_t;

#define IPREG_BAR_REG_54 0xa036
typedef struct {
	uint8_t spare1_trim_d2a: 8;
} ipreg_bar_reg_54_internal_t;

#define IPREG_BAR_REG_55 0xa037
typedef struct {
	uint8_t pads_tp_areg_trim_d2a: 5;
	uint8_t en_io_well_bias_vddio_comp_trim_d2a: 1;
	uint8_t en_pads_wellbias_rxnw_trim_d2a: 1;
	uint8_t en_pads_wellbias_rxpw_trim_d2a: 1;
} ipreg_bar_reg_55_internal_t;

#define IPREG_BAR_REG_56 0xa038
typedef struct {
	uint8_t en_pads_wellbias_txnw_trim_d2a: 1;
	uint8_t en_pads_wellbias_txpw_trim_d2a: 1;
	uint8_t pads_ap_cs_from_pad_disable_trim_d2a: 1;
	uint8_t pads_ap_cs_pe_trim_d2a: 1;
	uint8_t pads_ap_cs_pud_trim_d2a: 1;
	uint8_t pads_ap_sclk_from_pad_disable_trim_d2a: 1;
	uint8_t pads_ap_sclk_pe_trim_d2a: 1;
	uint8_t pads_ap_sclk_pud_trim_d2a: 1;
} ipreg_bar_reg_56_internal_t;

#define IPREG_BAR_REG_57 0xa039
typedef struct {
	uint8_t pads_ap_sdi_from_pad_disable_trim_d2a: 1;
	uint8_t pads_ap_sdi_pe_trim_d2a: 1;
	uint8_t pads_ap_sdi_pud_trim_d2a: 1;
	uint8_t pads_ap_sdo_from_pad_disable_trim_d2a: 1;
	uint8_t pads_ap_sdo_pe_trim_d2a: 1;
	uint8_t pads_ap_sdo_pud_trim_d2a: 1;
	uint8_t pads_aux_cs_tp3_from_pad_disable_trim_d2a: 1;
	uint8_t pads_aux_cs_tp3_tp_pe_trim_d2a: 1;
} ipreg_bar_reg_57_internal_t;

#define IPREG_BAR_REG_58 0xa03a
typedef struct {
	uint8_t pads_aux_cs_tp3_tp_pud_trim_d2a: 1;
	uint8_t pads_aux_sclk_tp2_from_pad_disable_trim_d2a: 1;
	uint8_t pads_aux_sclk_tp2_tp_pe_trim_d2a: 1;
	uint8_t pads_aux_sclk_tp2_tp_pud_trim_d2a: 1;
	uint8_t pads_aux_sdi_tp1_from_pad_disable_trim_d2a: 1;
	uint8_t pads_aux_sdi_tp1_tp_pe_trim_d2a: 1;
	uint8_t pads_aux_sdi_tp1_tp_pud_trim_d2a: 1;
	uint8_t pads_aux_sdo_from_pad_disable_trim_d2a: 1;
} ipreg_bar_reg_58_internal_t;

#define IPREG_BAR_REG_59 0xa03b
typedef struct {
	uint8_t pads_aux_sdo_pe_trim_d2a: 1;
	uint8_t pads_aux_sdo_pud_trim_d2a: 1;
	uint8_t pads_int1_tp0_from_pad_disable_trim_d2a: 1;
	uint8_t pads_int1_tp0_tp_pe_trim_d2a: 1;
	uint8_t pads_int1_tp0_tp_pud_trim_d2a: 1;
	uint8_t pads_int2_from_pad_disable_trim_d2a: 1;
	uint8_t pads_int2_pe_trim_d2a: 1;
	uint8_t pads_int2_pud_trim_d2a: 1;
} ipreg_bar_reg_59_internal_t;

#define IPREG_BAR_REG_60 0xa03c
typedef struct {
	uint8_t tmp_areg_trim_d2a: 5;
	uint8_t tmp_spare_trim_d2a: 3;
} ipreg_bar_reg_60_internal_t;

#define IPREG_BAR_REG_61 0xa03d
typedef struct {
	uint8_t tmp_dvbe_f_trim_d2a: 5;
	uint8_t resv_1: 3;
} ipreg_bar_reg_61_internal_t;

#define IPREG_BAR_REG_62 0xa03e
typedef struct {
	uint8_t tmp_vbe_f_trim_d2a: 5;
	uint8_t resv_1: 3;
} ipreg_bar_reg_62_internal_t;

#define IPREG_BAR_REG_63 0xa03f
typedef struct {
	uint8_t tmp_dvbe_c_trim_d2a: 4;
	uint8_t tmp_vbe_c_trim_d2a: 4;
} ipreg_bar_reg_63_internal_t;

#define IPREG_BAR_REG_64 0xa040
typedef struct {
	uint8_t dreg_dvdd_fine_trim_d2a: 4;
	uint8_t dreg_dvdd_coarse_trim_d2a: 3;
	uint8_t dreg_set_1meg_load_trim_d2a: 1;
} ipreg_bar_reg_64_internal_t;

#define IPREG_BAR_REG_65 0xa041
typedef struct {
	uint8_t dreg_sw_prech_trim_d2a: 3;
	uint8_t dreg_spare_trim_d2a: 2;
	uint8_t resv_1: 3;
} ipreg_bar_reg_65_internal_t;

#define IPREG_BAR_REG_66 0xa042
typedef struct {
	uint8_t gyro_cp25v_r2_trim_d2a: 4;
	uint8_t gyro_cp25v_cpclkdivrat_trim_d2a: 3;
	uint8_t en_gyro_cp25v_chop_trim_d2a: 1;
} ipreg_bar_reg_66_internal_t;

#define IPREG_BAR_REG_67 0xa043
typedef struct {
	uint8_t gyro_cp25v_chopclkdivrat_trim_d2a: 2;
	uint8_t gyro_cp25v_cprdytimer_trim_d2a: 2;
	uint8_t gyro_cp25v_spare_trim_d2a: 2;
	uint8_t gyro_cp25v_sync_ovrd_trim_d2a: 1;
	uint8_t resv_1: 1;
} ipreg_bar_reg_67_internal_t;

#define IPREG_BAR_REG_68 0xa044
typedef struct {
	uint8_t accel_vdrive_trim_d2a: 2;
	uint8_t resv_1: 6;
} ipreg_bar_reg_68_internal_t;

#define IPREG_BAR_REG_69 0xa045
typedef struct {
	uint8_t pads_od_pud_strength_trim_d2a: 2;
	uint8_t resv_1: 6;
} ipreg_bar_reg_69_internal_t;

#define IPREG_BAR_REG_70 0xa046
typedef struct {
	uint8_t accel_bias_spare_d2a: 8;
} ipreg_bar_reg_70_internal_t;

#define IPREG_BAR_REG_71 0xa047
typedef struct {
	uint8_t en_accel_drive_half_d2a: 1;
	uint8_t pd_accel_areg_b_d2a: 1;
	uint8_t pd_accel_cp3v_only_b_d2a: 1;
	uint8_t pd_accel_drivebuffer_b_d2a: 1;
	uint8_t pd_accel_iblend_b_d2a: 1;
	uint8_t pd_accel_vcmbuffer_b_d2a: 1;
	uint8_t pd_az_shield_wimpy_b_d2a: 1;
	uint8_t resv_1: 1;
} ipreg_bar_reg_71_internal_t;

#define IPREG_BAR_REG_72 0xa048
typedef struct {
	uint8_t accel_spare_d2a: 8;
} ipreg_bar_reg_72_internal_t;

#define IPREG_BAR_REG_73 0xa049
typedef struct {
	uint8_t en_ana_st_vref_d2a: 4;
	uint8_t accel_dig_tmode_cfg_d2a: 3;
	uint8_t accel_adc_fixed_ref_d2a: 1;
} ipreg_bar_reg_73_internal_t;

#define IPREG_BAR_REG_74 0xa04a
typedef struct {
	uint8_t accel_test_cfg_d2a: 3;
	uint8_t accel_testbuffer_gain_d2a: 3;
	uint8_t accel_drv_mode_d2a: 2;
} ipreg_bar_reg_74_internal_t;

#define IPREG_BAR_REG_75 0xa04b
typedef struct {
	uint8_t accel_xyz_sel_0_d2d: 2;
	uint8_t accel_xyz_sel_1_d2d: 2;
	uint8_t accel_xyz_sel_2_d2d: 2;
	uint8_t accel_cmos_only_d2a: 1;
	uint8_t accel_gaincal_en_d2a: 1;
} ipreg_bar_reg_75_internal_t;

#define IPREG_BAR_REG_76 0xa04c
typedef struct {
	uint8_t accel_gap_test_en_d2a: 1;
	uint8_t accel_integ1_out_ovrd_d2a: 1;
	uint8_t accel_integ2_out_ovrd_d2a: 1;
	uint8_t accel_vdrivecal_en_d2a: 1;
	uint8_t ax_drive_reset_off_d2a: 1;
	uint8_t axy_shldcal_en_d2a: 1;
	uint8_t ay_drive_reset_off_d2a: 1;
	uint8_t az_drive_reset_off_d2a: 1;
} ipreg_bar_reg_76_internal_t;

#define IPREG_BAR_REG_77 0xa04d
typedef struct {
	uint8_t en_accel_testbuffer_higain_d2a: 1;
	uint8_t en_accel_testbuffer_sw_pm1_d2a: 1;
	uint8_t en_accel_testbuffer_sw_pm2_d2a: 1;
	uint8_t en_accel_testbuffer_ugdrive_d2a: 1;
	uint8_t en_ana_st_adc_in_d2a: 1;
	uint8_t pd_accel_adc_b_d2a: 1;
	uint8_t pd_accel_adc_ref_b_d2a: 1;
	uint8_t pd_accel_c2v_b_d2a: 1;
} ipreg_bar_reg_77_internal_t;

#define IPREG_BAR_REG_78 0xa04e
typedef struct {
	uint8_t pd_accel_integ_b_d2a: 1;
	uint8_t pd_accel_testbuffer_b_d2a: 1;
	uint8_t pd_accel_testbuffer_hv_b_d2a: 1;
	uint8_t pd_ax_drive_b_d2a: 1;
	uint8_t pd_ay_drive_b_d2a: 1;
	uint8_t pd_az_drive_b_d2a: 1;
	uint8_t rst_accel_adc_b_d2a: 1;
	uint8_t rst_accel_arr_b_d2d: 1;
} ipreg_bar_reg_78_internal_t;

#define IPREG_BAR_REG_79 0xa04f
typedef struct {
	uint8_t rst_accel_c2v_b_d2a: 1;
	uint8_t rst_accel_integ1_b_d2a: 1;
	uint8_t rst_accel_integ2_b_d2a: 1;
	uint8_t rst_accel_testbuffer_b_d2a: 1;
	uint8_t rst_accel_testbuffer_dmd_b_d2a: 1;
	uint8_t resv_1: 3;
} ipreg_bar_reg_79_internal_t;

#define IPREG_BAR_REG_80 0xa050
typedef struct {
	uint8_t mbcore_spare_d2a: 4;
	uint8_t mb_areg_cp_clamp_dis_d2a: 1;
	uint8_t mbcore_rctrim_cap_sel_d2a: 1;
	uint8_t mbcore_rctrim_mode_d2a: 1;
	uint8_t pd_mb_areg_cp_b_d2a: 1;
} ipreg_bar_reg_80_internal_t;

#define IPREG_BAR_REG_81 0xa051
typedef struct {
	uint8_t pd_mb_vbgr_b_d2a: 1;
	uint8_t pd_mb_vbgr_test_b_d2a: 1;
	uint8_t pd_minibg_b_d2a: 1;
	uint8_t pd_rctrim_capmeas_b_d2a: 1;
	uint8_t pd_rctrim_resmeas_b_d2a: 1;
	uint8_t resv_1: 3;
} ipreg_bar_reg_81_internal_t;

#define IPREG_BAR_REG_82 0xa052
typedef struct {
	uint8_t spare1_d2a: 8;
} ipreg_bar_reg_82_internal_t;

#define IPREG_BAR_REG_83 0xa053
typedef struct {
	uint8_t io_well_bias_comp_sel_d2a: 1;
	uint8_t resv_1: 7;
} ipreg_bar_reg_83_internal_t;

#define IPREG_BAR_REG_84 0xa054
typedef struct {
	uint8_t en_ana_st_atp_d2a: 4;
	uint8_t io_char_filt_nofilt_sel_d2a: 1;
	uint8_t io_char_mux_sel_d2a: 1;
	uint8_t pads_aux_cs_tp3_abuf_en_d2a: 1;
	uint8_t pads_aux_cs_tp3_anl_sel_d2a: 1;
} ipreg_bar_reg_84_internal_t;

#define IPREG_BAR_REG_85 0xa055
typedef struct {
	uint8_t pads_aux_cs_tp3_atpbus_en_d2a: 1;
	uint8_t pads_aux_cs_tp3_dtpbus_en_d2a: 1;
	uint8_t pads_aux_cs_tp3_tp_aio_sel_d2a: 1;
	uint8_t pads_aux_sclk_tp2_abuf_en_d2a: 1;
	uint8_t pads_aux_sclk_tp2_anl_sel_d2a: 1;
	uint8_t pads_aux_sclk_tp2_atpbus_en_d2a: 1;
	uint8_t pads_aux_sclk_tp2_dtpbus_en_d2a: 1;
	uint8_t pads_aux_sclk_tp2_tp_aio_sel_d2a: 1;
} ipreg_bar_reg_85_internal_t;

#define IPREG_BAR_REG_86 0xa056
typedef struct {
	uint8_t pads_aux_sdi_tp1_abuf_en_d2a: 1;
	uint8_t pads_aux_sdi_tp1_anl_sel_d2a: 1;
	uint8_t pads_aux_sdi_tp1_atpbus_en_d2a: 1;
	uint8_t pads_aux_sdi_tp1_dtpbus_en_d2a: 1;
	uint8_t pads_aux_sdi_tp1_tp_aio_sel_d2a: 1;
	uint8_t pads_avdd_mon_d2a: 1;
	uint8_t pads_int1_tp0_abuf_en_d2a: 1;
	uint8_t pads_int1_tp0_anl_sel_d2a: 1;
} ipreg_bar_reg_86_internal_t;

#define IPREG_BAR_REG_87 0xa057
typedef struct {
	uint8_t pads_int1_tp0_atpbus_en_d2a: 1;
	uint8_t pads_int1_tp0_dtpbus_en_d2a: 1;
	uint8_t pads_int1_tp0_tp_aio_sel_d2a: 1;
	uint8_t pads_tp_lpbk_d2a: 1;
	uint8_t resv_1: 4;
} ipreg_bar_reg_87_internal_t;

#define IPREG_BAR_REG_88 0xa058
typedef struct {
	uint8_t tmp_spare_d2a: 4;
	uint8_t resv_1: 4;
} ipreg_bar_reg_88_internal_t;

#define IPREG_BAR_REG_89 0xa059
typedef struct {
	uint8_t scan_dreg_switch_on_d2a: 1;
	uint8_t scan_por_disable_d2a: 1;
	uint8_t resv_1: 6;
} ipreg_bar_reg_89_internal_t;

#define IPREG_BAR_REG_90 0xa05a
typedef struct {
	uint8_t gyro_cp25v_dft50n_d2a: 4;
	uint8_t gyro_cp25v_spare_d2a: 2;
	uint8_t resv_1: 2;
} ipreg_bar_reg_90_internal_t;

#define IPREG_BAR_REG_91 0xa05b
typedef struct {
	uint8_t gyro_dc2v_fine_analog_trim_d2a: 8;
} ipreg_bar_reg_91_internal_t;

#define IPREG_BAR_REG_92 0xa05c
typedef struct {
	uint8_t gyro_dc2v_gain_trim_d2a: 5;
	uint8_t gyro_dc2v_ib_trim_d2a: 3;
} ipreg_bar_reg_92_internal_t;

#define IPREG_BAR_REG_93 0xa05d
typedef struct {
	uint8_t gyro_dc2v_offset_trim_d2a: 4;
	uint8_t gyro_dc2v_ptatblend_trim_d2a: 4;
} ipreg_bar_reg_93_internal_t;

#define IPREG_BAR_REG_94 0xa05e
typedef struct {
	uint8_t gyro_dc2v_io_trim_d2a: 3;
	uint8_t gyro_dc2v_fet_trim_d2a: 2;
	uint8_t gyro_dc2v_rst_time_trim_d2a: 2;
	uint8_t resv_1: 1;
} ipreg_bar_reg_94_internal_t;

#define IPREG_BAR_REG_95 0xa05f
typedef struct {
	uint8_t gyro_dc2v_spare_trim_d2a: 2;
	uint8_t resv_1: 6;
} ipreg_bar_reg_95_internal_t;

#define IPREG_BAR_REG_96 0xa060
typedef struct {
	uint8_t gyro_drv_chpclk_sel_trim_d2a: 8;
} ipreg_bar_reg_96_internal_t;

#define IPREG_BAR_REG_97 0xa061
typedef struct {
	uint8_t gyro_drv_agc_amp_trim_d2a: 4;
	uint8_t gyro_drv_agc_gain_trim_d2a: 4;
} ipreg_bar_reg_97_internal_t;

#define IPREG_BAR_REG_98 0xa062
typedef struct {
	uint8_t gyro_drv_agc_lp_trim_d2a: 4;
	uint8_t gyro_drv_hvsup_trim_d2a: 4;
} ipreg_bar_reg_98_internal_t;

#define IPREG_BAR_REG_99 0xa063
typedef struct {
	uint8_t gyro_drv_spare1_trim_d2a: 4;
	uint8_t gyro_drv_clk_chop_sel_trim_d2a: 3;
	uint8_t gyro_drv_chop_disable_trim_d2a: 1;
} ipreg_bar_reg_99_internal_t;

#define IPREG_BAR_REG_100 0xa064
typedef struct {
	uint8_t gyro_drv_phs_freq_trim_d2a: 3;
	uint8_t gyro_drv_phs_gain_cm_trim_d2a: 3;
	uint8_t gyro_drv_clk_oscmon_timeout_trim_d2a: 2;
} ipreg_bar_reg_100_internal_t;

#define IPREG_BAR_REG_101 0xa065
typedef struct {
	uint8_t gyro_drv_phs_gain_diff_trim_d2a: 3;
	uint8_t gyro_drv_clk_oscmon_window_trim_d2a: 2;
	uint8_t gyro_drv_clk_chop_disable_trim_d2a: 1;
	uint8_t gyro_drv_clkst_2m_trim_d2a: 1;
	uint8_t gyro_drv_disable_suppswitch_trim_d2a: 1;
} ipreg_bar_reg_101_internal_t;

#define IPREG_BAR_REG_102 0xa066
typedef struct {
	uint8_t gyro_drv_dmdcomp_hi_cur_trim_d2a: 1;
	uint8_t gyro_drv_hvd_swap_polarity_trim_d2a: 1;
	uint8_t gyro_drv_overdrive_det_dis_trim_d2a: 1;
	uint8_t gyro_drv_phs_swap_polarity_trim_d2a: 1;
	uint8_t gyro_drv_pllref_rdy_threshold_trim_d2a: 1;
	uint8_t gyro_drv_prech_threshold_340m_trim_d2a: 1;
	uint8_t resv_1: 2;
} ipreg_bar_reg_102_internal_t;

#define IPREG_BAR_REG_103 0xa067
typedef struct {
	uint8_t spare2_trim_d2a: 8;
} ipreg_bar_reg_103_internal_t;

#define IPREG_BAR_REG_104 0xa068
typedef struct {
	uint8_t gx_sc2v_cc_hfs_trim_d2a: 7;
	uint8_t gx_sc2v_lco_trim_d2a: 1;
} ipreg_bar_reg_104_internal_t;

#define IPREG_BAR_REG_105 0xa069
typedef struct {
	uint8_t gx_sc2v_cc_lfs_trim_d2a: 7;
	uint8_t gx_sc2v_spare_trim_d2a: 1;
} ipreg_bar_reg_105_internal_t;

#define IPREG_BAR_REG_106 0xa06a
typedef struct {
	uint8_t gx_sc2v_ft_trim_d2a: 7;
	uint8_t gy_sc2v_lco_trim_d2a: 1;
} ipreg_bar_reg_106_internal_t;

#define IPREG_BAR_REG_107 0xa06b
typedef struct {
	uint8_t gx_sc2v_gain_hfs_trim_d2a: 7;
	uint8_t gy_sc2v_spare_trim_d2a: 1;
} ipreg_bar_reg_107_internal_t;

#define IPREG_BAR_REG_108 0xa06c
typedef struct {
	uint8_t gx_sc2v_gain_lfs_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_108_internal_t;

#define IPREG_BAR_REG_109 0xa06d
typedef struct {
	uint8_t gx_sc2v_soc_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_109_internal_t;

#define IPREG_BAR_REG_110 0xa06e
typedef struct {
	uint8_t gy_sc2v_cc_hfs_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_110_internal_t;

#define IPREG_BAR_REG_111 0xa06f
typedef struct {
	uint8_t gy_sc2v_cc_lfs_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_111_internal_t;

#define IPREG_BAR_REG_112 0xa070
typedef struct {
	uint8_t gy_sc2v_ft_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_112_internal_t;

#define IPREG_BAR_REG_113 0xa071
typedef struct {
	uint8_t gy_sc2v_gain_hfs_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_113_internal_t;

#define IPREG_BAR_REG_114 0xa072
typedef struct {
	uint8_t gy_sc2v_gain_lfs_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_114_internal_t;

#define IPREG_BAR_REG_115 0xa073
typedef struct {
	uint8_t gy_sc2v_soc_trim_d2a: 7;
	uint8_t resv_1: 1;
} ipreg_bar_reg_115_internal_t;

#define IPREG_BAR_REG_116 0xa074
typedef struct {
	uint8_t gx_sc2v_off_cm_trim_d2a: 4;
	uint8_t gx_sc2v_off_diff_trim_d2a: 4;
} ipreg_bar_reg_116_internal_t;

#define IPREG_BAR_REG_117 0xa075
typedef struct {
	uint8_t gx_sc2v_ptat2_hfs_trim_d2a: 4;
	uint8_t gx_sc2v_ptat2_lfs_trim_d2a: 4;
} ipreg_bar_reg_117_internal_t;

#define IPREG_BAR_REG_118 0xa076
typedef struct {
	uint8_t gx_sc2v_vbgr2_hfs_trim_d2a: 4;
	uint8_t gx_sc2v_vbgr2_lfs_trim_d2a: 4;
} ipreg_bar_reg_118_internal_t;

#define IPREG_BAR_REG_119 0xa077
typedef struct {
	uint8_t gy_sc2v_off_cm_trim_d2a: 4;
	uint8_t gy_sc2v_off_diff_trim_d2a: 4;
} ipreg_bar_reg_119_internal_t;

#define IPREG_BAR_REG_120 0xa078
typedef struct {
	uint8_t gy_sc2v_ptat2_hfs_trim_d2a: 4;
	uint8_t gy_sc2v_ptat2_lfs_trim_d2a: 4;
} ipreg_bar_reg_120_internal_t;

#define IPREG_BAR_REG_121 0xa079
typedef struct {
	uint8_t gy_sc2v_vbgr2_hfs_trim_d2a: 4;
	uint8_t gy_sc2v_vbgr2_lfs_trim_d2a: 4;
} ipreg_bar_reg_121_internal_t;

#define IPREG_BAR_REG_122 0xa07a
typedef struct {
	uint8_t gx_sc2v_ptat1_trim_d2a: 3;
	uint8_t gx_sc2v_vbgr1_trim_d2a: 3;
	uint8_t gx_sc2v_ostage_trim_d2a: 2;
} ipreg_bar_reg_122_internal_t;

#define IPREG_BAR_REG_123 0xa07b
typedef struct {
	uint8_t gy_sc2v_ptat1_trim_d2a: 3;
	uint8_t gy_sc2v_vbgr1_trim_d2a: 3;
	uint8_t gy_sc2v_ostage_trim_d2a: 2;
} ipreg_bar_reg_123_internal_t;

#define IPREG_BAR_REG_124 0xa07c
typedef struct {
	uint8_t gyro_pll_div_trim_d2a: 7;
	uint8_t en_gyro_pll_cpreplica_trim_d2a: 1;
} ipreg_bar_reg_124_internal_t;

#define IPREG_BAR_REG_125 0xa07d
typedef struct {
	uint8_t gyro_pll_nphi_trim_d2a: 7;
	uint8_t en_gyro_pll_dcc_cpreplica_trim_d2a: 1;
} ipreg_bar_reg_125_internal_t;

#define IPREG_BAR_REG_126 0xa07e
typedef struct {
	uint8_t gyro_pll_vfc_trim_d2a: 6;
	uint8_t resv_1: 2;
} ipreg_bar_reg_126_internal_t;

#define IPREG_BAR_REG_127 0xa07f
typedef struct {
	uint8_t rcosc_vfc_trim_d2a: 6;
	uint8_t resv_1: 2;
} ipreg_bar_reg_127_internal_t;

#define IPREG_BAR_REG_128 0xa080
typedef struct {
	uint8_t pll_vref_0p7v_trim_d2a: 5;
	uint8_t gyro_pll_dcc_icp_trim_d2a: 3;
} ipreg_bar_reg_128_internal_t;

#define IPREG_BAR_REG_129 0xa081
typedef struct {
	uint8_t rcosc_areg_trim_d2a: 5;
	uint8_t gyro_pll_icp_trim_d2a: 3;
} ipreg_bar_reg_129_internal_t;

#define IPREG_BAR_REG_130 0xa082
typedef struct {
	uint8_t gyro_pll_rcosc_spare_trim_d2a: 4;
	uint8_t gyro_pll_vco_trim_d2a: 4;
} ipreg_bar_reg_130_internal_t;

#define IPREG_BAR_REG_131 0xa083
typedef struct {
	uint8_t rcosc_vco_trim_d2a: 4;
	uint8_t gyro_pll_vco_tc_trim_d2a: 3;
	uint8_t resv_1: 1;
} ipreg_bar_reg_131_internal_t;

#define IPREG_BAR_REG_132 0xa084
typedef struct {
	uint8_t rcosc_vco_tc_trim_d2a: 3;
	uint8_t resv_1: 5;
} ipreg_bar_reg_132_internal_t;

#define IPREG_BAR_REG_133 0xa085
typedef struct {
	uint8_t gx_dmd_offset_hfs_trim_d2a: 5;
	uint8_t gx_dmd_mixer_lo_inv_trim_d2a: 1;
	uint8_t gy_dmd_mixer_lo_inv_trim_d2a: 1;
	uint8_t resv_1: 1;
} ipreg_bar_reg_133_internal_t;

#define IPREG_BAR_REG_134 0xa086
typedef struct {
	uint8_t gx_dmd_offset_lfs_trim_d2a: 5;
	uint8_t resv_1: 3;
} ipreg_bar_reg_134_internal_t;

#define IPREG_BAR_REG_135 0xa087
typedef struct {
	uint8_t gy_dmd_offset_hfs_trim_d2a: 5;
	uint8_t resv_1: 3;
} ipreg_bar_reg_135_internal_t;

#define IPREG_BAR_REG_136 0xa088
typedef struct {
	uint8_t gy_dmd_offset_lfs_trim_d2a: 5;
	uint8_t resv_1: 3;
} ipreg_bar_reg_136_internal_t;

#define IPREG_BAR_REG_137 0xa089
typedef struct {
	uint8_t gx_dmd_spare_trim_d2a: 4;
	uint8_t gx_integ_offset_trim_d2a: 4;
} ipreg_bar_reg_137_internal_t;

#define IPREG_BAR_REG_138 0xa08a
typedef struct {
	uint8_t gy_dmd_spare_trim_d2a: 4;
	uint8_t gy_integ_offset_trim_d2a: 4;
} ipreg_bar_reg_138_internal_t;

#define IPREG_BAR_REG_139 0xa08b
typedef struct {
	uint8_t gyro_dc2v_gp_analog_d2a: 2;
	uint8_t resv_1: 6;
} ipreg_bar_reg_139_internal_t;

#define IPREG_BAR_REG_140 0xa08c
typedef struct {
	uint8_t gyro_drv_pdb_mask_d2a: 8;
} ipreg_bar_reg_140_internal_t;

#define IPREG_BAR_REG_141 0xa08d
typedef struct {
	uint8_t gyro_drv_spare1_d2a: 8;
} ipreg_bar_reg_141_internal_t;

#define IPREG_BAR_REG_142 0xa08e
typedef struct {
	uint8_t gyro_drv_spare2_d2a: 8;
} ipreg_bar_reg_142_internal_t;

#define IPREG_BAR_REG_143 0xa08f
typedef struct {
	uint8_t gyro_drv_test_fsmforce_d2a: 8;
} ipreg_bar_reg_143_internal_t;

#define IPREG_BAR_REG_144 0xa090
typedef struct {
	uint8_t en_gyro_drv_dcc_test_d2a: 1;
	uint8_t gyro_drv_disable_oscdetector_d2a: 1;
	uint8_t gyro_drv_zero_force_d2a: 1;
	uint8_t resv_1: 5;
} ipreg_bar_reg_144_internal_t;

#define IPREG_BAR_REG_145 0xa091
typedef struct {
	uint8_t spare2_d2a: 8;
} ipreg_bar_reg_145_internal_t;

#define IPREG_BAR_REG_146 0xa092
typedef struct {
	uint8_t gx_sc2v_cfg_spare_d2a: 4;
	uint8_t gy_sc2v_cfg_spare_d2a: 4;
} ipreg_bar_reg_146_internal_t;

#define IPREG_BAR_REG_147 0xa093
typedef struct {
	uint8_t gx_sc2v_en_se_d2a: 2;
	uint8_t gy_sc2v_en_se_d2a: 2;
	uint8_t gx_sc2v_dc2v_ph_meas_d2a: 1;
	uint8_t gy_sc2v_dc2v_ph_meas_d2a: 1;
	uint8_t resv_1: 2;
} ipreg_bar_reg_147_internal_t;

#define IPREG_BAR_REG_148 0xa094
typedef struct {
	uint8_t dis_gyro_pllclk_d2a: 1;
	uint8_t dis_rcosc_oscclk_d2a: 1;
	uint8_t en_gyro_pll_refbuf_d2a: 1;
	uint8_t en_rcosc_calib_d2a: 1;
	uint8_t pll_vco_mode_d2a: 1;
	uint8_t sel_rcosc_plldiv_d2a: 1;
	uint8_t resv_1: 2;
} ipreg_bar_reg_148_internal_t;

#define IPREG_BAR_REG_149 0xa095
typedef struct {
	uint8_t gx_dmd_spare_d2a: 4;
	uint8_t gx_dmd_test_d2a: 4;
} ipreg_bar_reg_149_internal_t;

#define IPREG_BAR_REG_150 0xa096
typedef struct {
	uint8_t gy_dmd_spare_d2a: 4;
	uint8_t gy_dmd_test_d2a: 4;
} ipreg_bar_reg_150_internal_t;

#define IPREG_BAR_REG_151 0xa097
typedef struct {
	uint8_t pd_gx_adc_b_d2a: 1;
	uint8_t pd_gy_adc_b_d2a: 1;
	uint8_t resv_1: 6;
} ipreg_bar_reg_151_internal_t;

#define IPREG_BAR_REG_152 0xa098
typedef struct {
	uint8_t spare4_trim_d2a: 8;
} ipreg_bar_reg_152_internal_t;

#define IPREG_BAR_REG_153 0xa099
typedef struct {
	uint8_t gx_dmd_fine_phase_trim_d2d: 7;
	uint8_t gx_dmd_fine_phase_en_trim_d2d: 1;
} ipreg_bar_reg_153_internal_t;

#define IPREG_BAR_REG_154 0xa09a
typedef struct {
	uint8_t gx_dmd_phase_trim_d2d: 7;
	uint8_t gx_dmd_shift_trim_d2d: 1;
} ipreg_bar_reg_154_internal_t;

#define IPREG_BAR_REG_155 0xa09b
typedef struct {
	uint8_t gy_dmd_fine_phase_trim_d2d: 7;
	uint8_t gy_dmd_fine_phase_en_trim_d2d: 1;
} ipreg_bar_reg_155_internal_t;

#define IPREG_BAR_REG_156 0xa09c
typedef struct {
	uint8_t gy_dmd_phase_trim_d2d: 7;
	uint8_t gy_dmd_shift_trim_d2d: 1;
} ipreg_bar_reg_156_internal_t;

#define IPREG_BAR_REG_157 0xa09d
typedef struct {
	uint8_t gyro_dmd_divratio_trim_d2d: 7;
	uint8_t gyro_2lev_cont_mode_trim_d2d: 1;
} ipreg_bar_reg_157_internal_t;

#define IPREG_BAR_REG_158 0xa09e
typedef struct {
	uint8_t gyro_dmd_multilevels_n1_trim_d2d: 7;
	uint8_t gyro_dmd_lowres_trim_d2d: 1;
} ipreg_bar_reg_158_internal_t;

#define IPREG_BAR_REG_159 0xa09f
typedef struct {
	uint8_t gyro_dmd_multilevels_n2n1_trim_d2d: 7;
	uint8_t gyro_dmd_mix_short_trim_d2d: 1;
} ipreg_bar_reg_159_internal_t;

#define IPREG_BAR_REG_160 0xa0a0
typedef struct {
	uint8_t gyro_dmd_multilevels_n3n2_trim_d2d: 7;
	uint8_t gyro_dmd_punct_mode_trim_d2d: 1;
} ipreg_bar_reg_160_internal_t;

#define IPREG_BAR_REG_161 0xa0a1
typedef struct {
	uint8_t gz_dmd_fine_phase_trim_d2d: 7;
	uint8_t gz_dmd_fine_phase_en_trim_d2d: 1;
} ipreg_bar_reg_161_internal_t;

#define IPREG_BAR_REG_162 0xa0a2
typedef struct {
	uint8_t gz_dmd_phase_trim_d2d: 7;
	uint8_t gz_dmd_shift_trim_d2d: 1;
} ipreg_bar_reg_162_internal_t;

#define IPREG_BAR_REG_163 0xa0a3
typedef struct {
	uint8_t gdm_spare_trim_d2d: 4;
	uint8_t gyro_sc2v_swcap_ctrl_cfg_trim_d2d: 3;
	uint8_t resv_1: 1;
} ipreg_bar_reg_163_internal_t;

#define IPREG_BAR_REG_164 0xa0a4
typedef struct {
	uint8_t gx_dmd_phase_quadrant_trim_d2d: 2;
	uint8_t gy_dmd_phase_quadrant_trim_d2d: 2;
	uint8_t gyro_dmd_fine_phase_width_trim_d2d: 2;
	uint8_t gyro_dmd_lvls_mode_trim_d2d: 2;
} ipreg_bar_reg_164_internal_t;

#define IPREG_BAR_REG_165 0xa0a5
typedef struct {
	uint8_t gyro_sc2v_rst_ctrl_trim_d2d: 2;
	uint8_t gyro_sc2v_rst_width_trim_d2d: 2;
	uint8_t gz_dmd_phase_quadrant_trim_d2d: 2;
	uint8_t resv_1: 2;
} ipreg_bar_reg_165_internal_t;

#define IPREG_BAR_REG_166 0xa0a6
typedef struct {
	uint8_t gyro_adc_rc_trim_d2a: 4;
	uint8_t gyro_adc_chop_trim_d2a: 3;
	uint8_t en_gyro_adc_dem_seed_trim_d2a: 1;
} ipreg_bar_reg_166_internal_t;

#define IPREG_BAR_REG_167 0xa0a7
typedef struct {
	uint8_t gyro_adc_dither_trim_d2a: 2;
	uint8_t en_gyro_adc_dem_trim_d2a: 1;
	uint8_t en_gyro_mix_res_trim_d2a: 1;
	uint8_t gyro_adc_in_res_trim_d2a: 1;
	uint8_t gyro_mix_levels_trim_d2a: 1;
	uint8_t resv_1: 2;
} ipreg_bar_reg_167_internal_t;

#define IPREG_BAR_REG_168 0xa0a8
typedef struct {
	uint8_t gyro_sc2v_grfb_trim_d2a: 2;
	uint8_t gyro_sc2v_rrst_trim_d2a: 2;
	uint8_t gyro_en_b_rsw_filt_trim_d2a: 1;
	uint8_t gyro_sc2v_dis_prt_trim_d2a: 1;
	uint8_t gyro_sc2v_sw_mode_b_trim_d2a: 1;
	uint8_t resv_1: 1;
} ipreg_bar_reg_168_internal_t;

#define IPREG_BAR_REG_169 0xa0a9
typedef struct {
	uint8_t spare4_d2a: 8;
} ipreg_bar_reg_169_internal_t;

#define IPREG_BAR_REG_170 0xa0aa
typedef struct {
	uint8_t gx_dmd_fine_phase_shape_d2d: 8;
} ipreg_bar_reg_170_internal_t;

#define IPREG_BAR_REG_171 0xa0ab
typedef struct {
	uint8_t gy_dmd_fine_phase_shape_d2d: 8;
} ipreg_bar_reg_171_internal_t;

#define IPREG_BAR_REG_172 0xa0ac
typedef struct {
	uint8_t gz_dmd_fine_phase_shape_d2d: 8;
} ipreg_bar_reg_172_internal_t;

#define IPREG_BAR_REG_173 0xa0ad
typedef struct {
	uint8_t gdm_spare_d2d: 4;
	uint8_t rst_gx_sc2v_b_mode_d2d: 2;
	uint8_t rst_gy_sc2v_b_mode_d2d: 2;
} ipreg_bar_reg_173_internal_t;

#define IPREG_BAR_REG_174 0xa0ae
typedef struct {
	uint8_t rst_gz_sc2v_b_mode_d2d: 2;
	uint8_t gyro_dmd_dis_clk_d2d: 1;
	uint8_t resv_1: 5;
} ipreg_bar_reg_174_internal_t;

#define IPREG_BAR_REG_175 0xa0af
typedef struct {
	uint8_t pd_gyro_ana_mixer_b_d2a: 1;
	uint8_t resv_1: 7;
} ipreg_bar_reg_175_internal_t;

#define IPREG_BAR_REG_176 0xa0b0
typedef struct {
	uint8_t gyro_sc2v_en_hr_d2a: 1;
	uint8_t resv_1: 7;
} ipreg_bar_reg_176_internal_t;

/* IPREG_ANA */
#define IPREG_ANA_REG_0 0xa300
typedef struct {
	uint8_t gyro_sc2v_cont_mode_b_trim_d2a: 1;
	uint8_t resv_1: 7;
} ipreg_ana_reg_0_internal_t;

#define IPREG_ANA_REG_1 0xa301
#define IPREG_ANA_REG_2 0xa302
#define IPREG_ANA_REG_3 0xa303
typedef struct {
	uint8_t pd_mbcore_rctrim_b_d2a: 1;
	uint8_t resv_1: 7;
} ipreg_ana_reg_3_internal_t;

#define IPREG_ANA_REG_4 0xa304
typedef struct {
	uint8_t gyro_drv_test_fsmread_a2d: 8;
} ipreg_ana_reg_4_internal_t;

#define IPREG_ANA_REG_5 0xa305
typedef struct {
	uint8_t gyro_drv_xst_mode_d2a: 2;
	uint8_t gyro_drv_yst_mode_d2a: 2;
	uint8_t gyro_drv_zst_mode_d2a: 2;
	uint8_t resv_1: 2;
} ipreg_ana_reg_5_internal_t;

#define IPREG_ANA_REG_6 0xa306
typedef struct {
	uint8_t gyro_drv_xst_pol_d2a: 1;
	uint8_t gyro_drv_yst_pol_d2a: 1;
	uint8_t gyro_drv_zst_pol_d2a: 1;
	uint8_t resv_1: 5;
} ipreg_ana_reg_6_internal_t;

#define IPREG_ANA_REG_7 0xa307
typedef struct {
	uint8_t az_negcp_valid_a2d: 1;
	uint8_t resv_1: 7;
} ipreg_ana_reg_7_internal_t;

#define IPREG_ANA_REG_8 0xa308
typedef struct {
	uint8_t pd_accel_cp45_st_b_d2a: 1;
	uint8_t pd_accel_st_b_d2a: 1;
	uint8_t resv_1: 6;
} ipreg_ana_reg_8_internal_t;

#define IPREG_ANA_REG_9 0xa309
typedef struct {
	uint8_t en_gyro_pll_refclk_d2a: 1;
	uint8_t resv_1: 7;
} ipreg_ana_reg_9_internal_t;

#define IPREG_ANA_REG_10 0xa30a
typedef struct {
	uint8_t pads_aux_cs_tp3_tp_en_d2a: 1;
	uint8_t pads_aux_sclk_tp2_tp_en_d2a: 1;
	uint8_t pads_aux_sdi_tp1_tp_en_d2a: 1;
	uint8_t pads_int1_tp0_tp_en_d2a: 1;
	uint8_t resv_1: 4;
} ipreg_ana_reg_10_internal_t;

#define IPREG_ANA_REG_11 0xa30b
typedef struct {
	uint8_t pd_pads_tp_avdd_b_d2a: 1;
	uint8_t resv_1: 7;
} ipreg_ana_reg_11_internal_t;

#define ATMD_MISC1 0xa30c
typedef struct {
	uint8_t atmd_addr_d2d: 6;
	uint8_t atmd_data_d2d: 1;
	uint8_t resv_1: 1;
} atmd_misc1_internal_t;

#define ATMD_MISC2 0xa30d
typedef struct {
	uint8_t atmd_mode_d2d: 4;
	uint8_t atmd_mode_en_d2d: 1;
	uint8_t atmd_reset_b_d2d: 1;
	uint8_t atmd_strobe_d2d: 1;
	uint8_t resv_1: 1;
} atmd_misc2_internal_t;

#define ATMD_MISC3 0xa30e
typedef struct {
	uint8_t resv_1: 7;
	uint8_t atmd_bit_d2d: 1;
} atmd_misc3_internal_t;

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
typedef struct {
	uint8_t self_cal_ft_ver: 8;
} imem_sram_reg_24_internal_t;

#define MANU_ID 0x19
typedef struct {
	uint8_t manu_id: 8;
} manu_id_internal_t;

#define CHIP_ID 0x1a
typedef struct {
	uint8_t chip_id: 8;
} chip_id_internal_t;

#define REV_ID 0x1b
typedef struct {
	uint8_t rev_id: 8;
} rev_id_internal_t;

#define ASIC_WAFER_ID 0x1c
typedef struct {
	uint8_t asic_wafer_id: 8;
} asic_wafer_id_internal_t;

#define ASIC_DIE_ID_0 0x1d
typedef struct {
	uint8_t asic_die_id_1: 8;
} asic_die_id_0_internal_t;

#define ASIC_DIE_ID_1 0x1e
typedef struct {
	uint8_t asic_die_id_2: 8;
} asic_die_id_1_internal_t;

#define MEMS_WAFER_ID 0x1f
typedef struct {
	uint8_t mems_wafer_id: 8;
} mems_wafer_id_internal_t;

#define MEMS_DIE_ID_0 0x20
typedef struct {
	uint8_t mems_die_id_1: 8;
} mems_die_id_0_internal_t;

#define MEMS_DIE_ID_1 0x21
typedef struct {
	uint8_t mems_die_id_2: 8;
} mems_die_id_1_internal_t;

#define SW_RCOSC_TRIM 0x22
typedef struct {
	uint8_t sw_rcosc_trim: 8;
} sw_rcosc_trim_internal_t;

#define SW_PLL_TRIM 0x23
typedef struct {
	uint8_t sw_pll_trim: 8;
} sw_pll_trim_internal_t;

#define SA_KEY_LAST_BYTE 0x24
typedef struct {
	uint8_t sa_key_last_byte: 8;
} sa_key_last_byte_internal_t;

/* IMEM_SRAM_APEX */
#define IMEM_SRAM_APEX_REG_42 0x2a
#define IMEM_SRAM_APEX_REG_43 0x2b
#define IMEM_SRAM_APEX_REG_44 0x2c
#define IMEM_SRAM_APEX_REG_45 0x2d
#define IMEM_SRAM_APEX_REG_46 0x2e
#define IMEM_SRAM_APEX_REG_47 0x2f
#define IMEM_SRAM_APEX_REG_48 0x30
#define IMEM_SRAM_APEX_REG_49 0x31
#define IMEM_SRAM_APEX_REG_50 0x32
#define IMEM_SRAM_APEX_REG_51 0x33
#define IMEM_SRAM_APEX_REG_52 0x34
#define IMEM_SRAM_APEX_REG_53 0x35
#define IMEM_SRAM_APEX_REG_54 0x36
#define IMEM_SRAM_APEX_REG_55 0x37
#define IMEM_SRAM_APEX_REG_56 0x38
#define IMEM_SRAM_APEX_REG_57 0x39
#define IMEM_SRAM_APEX_REG_58 0x3a
#define IMEM_SRAM_APEX_REG_59 0x3b
#define IMEM_SRAM_APEX_REG_60 0x3c
#define IMEM_SRAM_APEX_REG_61 0x3d
#define IMEM_SRAM_APEX_REG_62 0x3e
#define IMEM_SRAM_APEX_REG_63 0x3f
#define IMEM_SRAM_APEX_REG_98 0x62
typedef struct {
	uint8_t quat_reset_en: 8;
} imem_sram_apex_reg_98_internal_t;

#define IMEM_SRAM_APEX_REG_99 0x63
typedef struct {
	uint8_t tilt_reset_en: 8;
} imem_sram_apex_reg_99_internal_t;

#define IMEM_SRAM_APEX_REG_121 0x79
typedef struct {
	uint8_t ped_step_cadence: 8;
} imem_sram_apex_reg_121_internal_t;

#define IMEM_SRAM_APEX_REG_122 0x7a
typedef struct {
	uint8_t ped_activity_class: 8;
} imem_sram_apex_reg_122_internal_t;

#define IMEM_SRAM_APEX_REG_124 0x7c
#define IMEM_SRAM_APEX_REG_125 0x7d
#define IMEM_SRAM_APEX_REG_126 0x7e
#define IMEM_SRAM_APEX_REG_127 0x7f
#define IMEM_SRAM_APEX_REG_136 0x88
#define IMEM_SRAM_APEX_REG_137 0x89
#define IMEM_SRAM_APEX_REG_138 0x8a
#define IMEM_SRAM_APEX_REG_139 0x8b
#define IMEM_SRAM_APEX_REG_142 0x8e
typedef struct {
	uint8_t tap_num: 8;
} imem_sram_apex_reg_142_internal_t;

#define IMEM_SRAM_APEX_REG_143 0x8f
typedef struct {
	uint8_t tap_axis: 8;
} imem_sram_apex_reg_143_internal_t;

#define IMEM_SRAM_APEX_REG_144 0x90
typedef struct {
	uint8_t tap_dir: 8;
} imem_sram_apex_reg_144_internal_t;

#define IMEM_SRAM_APEX_REG_145 0x91
typedef struct {
	uint8_t double_tap_timing: 8;
} imem_sram_apex_reg_145_internal_t;

#define IMEM_SRAM_APEX_REG_146 0x92
typedef struct {
	uint8_t triple_tap_timing: 8;
} imem_sram_apex_reg_146_internal_t;

#define IMEM_SRAM_APEX_REG_147 0x93
typedef struct {
	uint8_t shake_outaxis: 8;
} imem_sram_apex_reg_147_internal_t;

#define IMEM_SRAM_APEX_REG_148 0x94
typedef struct {
	uint8_t nomotion_outlock: 8;
} imem_sram_apex_reg_148_internal_t;

#define IMEM_SRAM_APEX_REG_149 0x95
typedef struct {
	uint8_t nomotion_majority_axis: 8;
} imem_sram_apex_reg_149_internal_t;

#define IMEM_SRAM_APEX_REG_150 0x96
typedef struct {
	uint8_t nomotion_sign_angle: 8;
} imem_sram_apex_reg_150_internal_t;

#define IMEM_SRAM_APEX_REG_152 0x98
#define IMEM_SRAM_APEX_REG_153 0x99
#define IMEM_SRAM_APEX_REG_154 0x9a
#define IMEM_SRAM_APEX_REG_155 0x9b
#define IMEM_SRAM_APEX_REG_156 0x9c
#define IMEM_SRAM_APEX_REG_157 0x9d
#define IMEM_SRAM_APEX_REG_168 0xa8
#define IMEM_SRAM_APEX_REG_169 0xa9
#define IMEM_SRAM_APEX_REG_170 0xaa
#define IMEM_SRAM_APEX_REG_171 0xab
#define IMEM_SRAM_APEX_REG_188 0xbc
#define IMEM_SRAM_APEX_REG_189 0xbd
#define IMEM_SRAM_APEX_REG_190 0xbe
#define IMEM_SRAM_APEX_REG_191 0xbf
#define IMEM_SRAM_APEX_REG_192 0xc0
#define IMEM_SRAM_APEX_REG_193 0xc1
#define IMEM_SRAM_APEX_REG_194 0xc2
#define IMEM_SRAM_APEX_REG_195 0xc3
#define IMEM_SRAM_APEX_REG_196 0xc4
#define IMEM_SRAM_APEX_REG_197 0xc5
#define IMEM_SRAM_APEX_REG_198 0xc6
#define IMEM_SRAM_APEX_REG_199 0xc7
#define IMEM_SRAM_APEX_REG_206 0xce
#define IMEM_SRAM_APEX_REG_207 0xcf
#define IMEM_SRAM_APEX_REG_208 0xd0
#define IMEM_SRAM_APEX_REG_209 0xd1
#define IMEM_SRAM_APEX_REG_210 0xd2
#define IMEM_SRAM_APEX_REG_211 0xd3
#define IMEM_SRAM_APEX_REG_218 0xda
#define IMEM_SRAM_APEX_REG_219 0xdb
#define IMEM_SRAM_APEX_REG_220 0xdc
#define IMEM_SRAM_APEX_REG_221 0xdd
#define IMEM_SRAM_APEX_REG_222 0xde
#define IMEM_SRAM_APEX_REG_223 0xdf
#define IMEM_SRAM_APEX_REG_228 0xe4
#define IMEM_SRAM_APEX_REG_229 0xe5
#define IMEM_SRAM_APEX_REG_230 0xe6
typedef struct {
	uint8_t tap_tmin: 8;
} imem_sram_apex_reg_230_internal_t;

#define IMEM_SRAM_APEX_REG_231 0xe7
typedef struct {
	uint8_t tap_min_jerk: 8;
} imem_sram_apex_reg_231_internal_t;

#define IMEM_SRAM_APEX_REG_232 0xe8
typedef struct {
	uint8_t tap_smudge_reject_thr: 8;
} imem_sram_apex_reg_232_internal_t;

#define IMEM_SRAM_APEX_REG_233 0xe9
typedef struct {
	uint8_t tap_max_peak_tol: 8;
} imem_sram_apex_reg_233_internal_t;

#define IMEM_SRAM_APEX_REG_234 0xea
typedef struct {
	uint8_t tap_tavg: 8;
} imem_sram_apex_reg_234_internal_t;

#define IMEM_SRAM_APEX_REG_236 0xec
typedef struct {
	uint8_t tap_max_tap: 8;
} imem_sram_apex_reg_236_internal_t;

#define IMEM_SRAM_APEX_REG_237 0xed
typedef struct {
	uint8_t tap_min_tap: 8;
} imem_sram_apex_reg_237_internal_t;

#define IMEM_SRAM_APEX_REG_238 0xee
#define IMEM_SRAM_APEX_REG_239 0xef
#define IMEM_SRAM_APEX_REG_240 0xf0
#define IMEM_SRAM_APEX_REG_241 0xf1
#define IMEM_SRAM_APEX_REG_242 0xf2
typedef struct {
	uint8_t tap_axis_select_mask: 8;
} imem_sram_apex_reg_242_internal_t;

#define IMEM_SRAM_APEX_REG_364 0x16c
#define IMEM_SRAM_APEX_REG_365 0x16d
#define IMEM_SRAM_APEX_REG_366 0x16e
#define IMEM_SRAM_APEX_REG_367 0x16f
#define IMEM_SRAM_APEX_REG_368 0x170
#define IMEM_SRAM_APEX_REG_369 0x171
#define IMEM_SRAM_APEX_REG_370 0x172
#define IMEM_SRAM_APEX_REG_371 0x173
#define IMEM_SRAM_APEX_REG_372 0x174
#define IMEM_SRAM_APEX_REG_373 0x175
#define IMEM_SRAM_APEX_REG_374 0x176
#define IMEM_SRAM_APEX_REG_375 0x177
#define IMEM_SRAM_APEX_REG_376 0x178
#define IMEM_SRAM_APEX_REG_377 0x179
#define IMEM_SRAM_APEX_REG_378 0x17a
#define IMEM_SRAM_APEX_REG_379 0x17b
#define IMEM_SRAM_APEX_REG_380 0x17c
#define IMEM_SRAM_APEX_REG_381 0x17d
#define IMEM_SRAM_APEX_REG_382 0x17e
#define IMEM_SRAM_APEX_REG_383 0x17f
#define IMEM_SRAM_APEX_REG_384 0x180
#define IMEM_SRAM_APEX_REG_385 0x181
#define IMEM_SRAM_APEX_REG_386 0x182
#define IMEM_SRAM_APEX_REG_387 0x183
#define IMEM_SRAM_APEX_REG_388 0x184
#define IMEM_SRAM_APEX_REG_389 0x185
#define IMEM_SRAM_APEX_REG_390 0x186
#define IMEM_SRAM_APEX_REG_391 0x187
#define IMEM_SRAM_APEX_REG_392 0x188
#define IMEM_SRAM_APEX_REG_393 0x189
#define IMEM_SRAM_APEX_REG_394 0x18a
#define IMEM_SRAM_APEX_REG_395 0x18b
#define IMEM_SRAM_APEX_REG_396 0x18c
#define IMEM_SRAM_APEX_REG_397 0x18d
#define IMEM_SRAM_APEX_REG_398 0x18e
#define IMEM_SRAM_APEX_REG_399 0x18f
#define IMEM_SRAM_APEX_REG_400 0x190
#define IMEM_SRAM_APEX_REG_401 0x191
#define IMEM_SRAM_APEX_REG_402 0x192
#define IMEM_SRAM_APEX_REG_403 0x193
#define IMEM_SRAM_APEX_REG_404 0x194
#define IMEM_SRAM_APEX_REG_405 0x195
#define IMEM_SRAM_APEX_REG_406 0x196
#define IMEM_SRAM_APEX_REG_407 0x197
#define IMEM_SRAM_APEX_REG_408 0x198
#define IMEM_SRAM_APEX_REG_409 0x199
#define IMEM_SRAM_APEX_REG_410 0x19a
#define IMEM_SRAM_APEX_REG_411 0x19b
#define IMEM_SRAM_APEX_REG_492 0x1ec
#define IMEM_SRAM_APEX_REG_493 0x1ed
#define IMEM_SRAM_APEX_REG_494 0x1ee
#define IMEM_SRAM_APEX_REG_495 0x1ef
#define IMEM_SRAM_APEX_REG_532 0x214
#define IMEM_SRAM_APEX_REG_533 0x215
#define IMEM_SRAM_APEX_REG_536 0x218
#define IMEM_SRAM_APEX_REG_537 0x219
#define IMEM_SRAM_APEX_REG_538 0x21a
#define IMEM_SRAM_APEX_REG_539 0x21b
#define IMEM_SRAM_APEX_REG_764 0x2fc
#define IMEM_SRAM_APEX_REG_765 0x2fd
#define IMEM_SRAM_APEX_REG_894 0x37e
#define IMEM_SRAM_APEX_REG_895 0x37f
#define IMEM_SRAM_APEX_REG_896 0x380
#define IMEM_SRAM_APEX_REG_897 0x381
#define IMEM_SRAM_APEX_REG_898 0x382
#define IMEM_SRAM_APEX_REG_899 0x383
#define IMEM_SRAM_APEX_REG_900 0x384
#define IMEM_SRAM_APEX_REG_901 0x385
#define IMEM_SRAM_APEX_REG_902 0x386
#define IMEM_SRAM_APEX_REG_903 0x387
#define IMEM_SRAM_APEX_REG_904 0x388
#define IMEM_SRAM_APEX_REG_905 0x389
#define IMEM_SRAM_APEX_REG_906 0x38a
#define IMEM_SRAM_APEX_REG_907 0x38b
#define IMEM_SRAM_APEX_REG_908 0x38c
#define IMEM_SRAM_APEX_REG_909 0x38d
#define IMEM_SRAM_APEX_REG_910 0x38e
#define IMEM_SRAM_APEX_REG_911 0x38f
#define IMEM_SRAM_APEX_REG_912 0x390
typedef struct {
	uint8_t ped_sensitivity_mode: 8;
} imem_sram_apex_reg_912_internal_t;

#define IMEM_SRAM_APEX_REG_938 0x3aa
typedef struct {
	uint8_t smd_sensitivity: 8;
} imem_sram_apex_reg_938_internal_t;

#define IMEM_SRAM_APEX_REG_1068 0x42c
typedef struct {
	uint8_t b2s_mounting_matrix: 8;
} imem_sram_apex_reg_1068_internal_t;

#define IMEM_SRAM_APEX_REG_1070 0x42e
#define IMEM_SRAM_APEX_REG_1071 0x42f
#define IMEM_SRAM_APEX_REG_1072 0x430
#define IMEM_SRAM_APEX_REG_1073 0x431
#define IMEM_SRAM_APEX_REG_1074 0x432
#define IMEM_SRAM_APEX_REG_1075 0x433
#define IMEM_SRAM_APEX_REG_1076 0x434
#define IMEM_SRAM_APEX_REG_1077 0x435
#define IMEM_SRAM_APEX_REG_1078 0x436
#define IMEM_SRAM_APEX_REG_1079 0x437
#define IMEM_SRAM_APEX_REG_1080 0x438
#define IMEM_SRAM_APEX_REG_1081 0x439
#define IMEM_SRAM_APEX_REG_1082 0x43a
#define IMEM_SRAM_APEX_REG_1083 0x43b
#define IMEM_SRAM_APEX_REG_1084 0x43c
#define IMEM_SRAM_APEX_REG_1085 0x43d
#define IMEM_SRAM_APEX_REG_1086 0x43e
#define IMEM_SRAM_APEX_REG_1087 0x43f
#define IMEM_SRAM_APEX_REG_1088 0x440
#define IMEM_SRAM_APEX_REG_1089 0x441
#define IMEM_SRAM_APEX_REG_1090 0x442
#define IMEM_SRAM_APEX_REG_1091 0x443
#define IMEM_SRAM_APEX_REG_1092 0x444
#define IMEM_SRAM_APEX_REG_1093 0x445
#define IMEM_SRAM_APEX_REG_1094 0x446
#define IMEM_SRAM_APEX_REG_1095 0x447
#define IMEM_SRAM_APEX_REG_1096 0x448
#define IMEM_SRAM_APEX_REG_1097 0x449
#define IMEM_SRAM_APEX_REG_1098 0x44a
#define IMEM_SRAM_APEX_REG_1099 0x44b
#define IMEM_SRAM_APEX_REG_1100 0x44c
#define IMEM_SRAM_APEX_REG_1101 0x44d
#define IMEM_SRAM_APEX_REG_1102 0x44e
#define IMEM_SRAM_APEX_REG_1103 0x44f
#define IMEM_SRAM_APEX_REG_1104 0x450
#define IMEM_SRAM_APEX_REG_1105 0x451
#define IMEM_SRAM_APEX_REG_1106 0x452
#define IMEM_SRAM_APEX_REG_1107 0x453
#define IMEM_SRAM_APEX_REG_1108 0x454
#define IMEM_SRAM_APEX_REG_1109 0x455
#define IMEM_SRAM_APEX_REG_1110 0x456
#define IMEM_SRAM_APEX_REG_1111 0x457
#define IMEM_SRAM_APEX_REG_1112 0x458
#define IMEM_SRAM_APEX_REG_1113 0x459
#define IMEM_SRAM_APEX_REG_1114 0x45a
#define IMEM_SRAM_APEX_REG_1115 0x45b
#define IMEM_SRAM_APEX_REG_1116 0x45c
#define IMEM_SRAM_APEX_REG_1117 0x45d
#define IMEM_SRAM_APEX_REG_1118 0x45e
#define IMEM_SRAM_APEX_REG_1119 0x45f
#define IMEM_SRAM_APEX_REG_1280 0x500
#define IMEM_SRAM_APEX_REG_1281 0x501
#define IMEM_SRAM_APEX_REG_1282 0x502
#define IMEM_SRAM_APEX_REG_1283 0x503
#define IMEM_SRAM_APEX_REG_1284 0x504
#define IMEM_SRAM_APEX_REG_1285 0x505
#define IMEM_SRAM_APEX_REG_1286 0x506
#define IMEM_SRAM_APEX_REG_1287 0x507
#define IMEM_SRAM_APEX_REG_1288 0x508
#define IMEM_SRAM_APEX_REG_1289 0x509
#define IMEM_SRAM_APEX_REG_1290 0x50a
#define IMEM_SRAM_APEX_REG_1291 0x50b
#define IMEM_SRAM_APEX_REG_1292 0x50c
#define IMEM_SRAM_APEX_REG_1293 0x50d
#define IMEM_SRAM_APEX_REG_1294 0x50e
#define IMEM_SRAM_APEX_REG_1295 0x50f
#define IMEM_SRAM_APEX_REG_1296 0x510
#define IMEM_SRAM_APEX_REG_1297 0x511
#define IMEM_SRAM_APEX_REG_1298 0x512
#define IMEM_SRAM_APEX_REG_1299 0x513
#define IMEM_SRAM_APEX_REG_1300 0x514
#define IMEM_SRAM_APEX_REG_1301 0x515
#define IMEM_SRAM_APEX_REG_1314 0x522
#define IMEM_SRAM_APEX_REG_1315 0x523
#define IMEM_SRAM_APEX_REG_1316 0x524
#define IMEM_SRAM_APEX_REG_1317 0x525
#define IMEM_SRAM_APEX_REG_1318 0x526
#define IMEM_SRAM_APEX_REG_1319 0x527
#define IMEM_SRAM_APEX_REG_1364 0x554
#define IMEM_SRAM_APEX_REG_1365 0x555
#define IMEM_SRAM_APEX_REG_1366 0x556
#define IMEM_SRAM_APEX_REG_1367 0x557
#define IMEM_SRAM_APEX_REG_1396 0x574
#define IMEM_SRAM_APEX_REG_1397 0x575
#define IMEM_SRAM_APEX_REG_1398 0x576
#define IMEM_SRAM_APEX_REG_1399 0x577
#define IMEM_SRAM_APEX_REG_1400 0x578
#define IMEM_SRAM_APEX_REG_1401 0x579
#define IMEM_SRAM_APEX_REG_1402 0x57a
#define IMEM_SRAM_APEX_REG_1403 0x57b
#define IMEM_SRAM_APEX_REG_1404 0x57c
#define IMEM_SRAM_APEX_REG_1405 0x57d
#define IMEM_SRAM_APEX_REG_1406 0x57e
#define IMEM_SRAM_APEX_REG_1407 0x57f
#define IMEM_SRAM_APEX_REG_1408 0x580
#define IMEM_SRAM_APEX_REG_1409 0x581
#define IMEM_SRAM_APEX_REG_1410 0x582
#define IMEM_SRAM_APEX_REG_1411 0x583
#define IMEM_SRAM_APEX_REG_1412 0x584
#define IMEM_SRAM_APEX_REG_1413 0x585
#define IMEM_SRAM_APEX_REG_1414 0x586
#define IMEM_SRAM_APEX_REG_1415 0x587
#define IMEM_SRAM_APEX_REG_1416 0x588
#define IMEM_SRAM_APEX_REG_1417 0x589
#define IMEM_SRAM_APEX_REG_1418 0x58a
#define IMEM_SRAM_APEX_REG_1419 0x58b

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
} mstr_spi_ctrl_internal_t;

#define MSTR_SPI_ADDR 0xa601
typedef struct {
	uint8_t mstr_spi_addr: 8;
} mstr_spi_addr_internal_t;

#define MSTR_SPI_WR_DATA 0xa602
typedef struct {
	uint8_t mstr_spi_wr_data: 8;
} mstr_spi_wr_data_internal_t;

#define MSTR_SPI_RD_DATA 0xa603
typedef struct {
	uint8_t mstr_spi_rd_data: 8;
} mstr_spi_rd_data_internal_t;

#define MSTR_SPI_STATUS 0xa604
typedef struct {
	uint8_t mstr_spi_idle: 1;
	uint8_t mstr_spi_done: 1;
	uint8_t resv_1: 6;
} mstr_spi_status_internal_t;

#define SENSOR_DATA_CTRL 0xa605
typedef struct {
	uint8_t start_sensor: 1;
	uint8_t prologue_en: 1;
	uint8_t acc_sensor_data_en: 1;
	uint8_t gyro_sensor_data_en: 1;
	uint8_t resv_1: 4;
} sensor_data_ctrl_internal_t;

#define FPGA_ID 0xa606
typedef struct {
	uint8_t fpga_id: 8;
} fpga_id_internal_t;

#define FPGA_CONFIG 0xa607
typedef struct {
	uint8_t fpga_root_clk_sel: 1;
	uint8_t resv_1: 7;
} fpga_config_internal_t;

#define REVISION1 0xa608
typedef struct {
	uint8_t fpga_major_revision: 8;
} revision1_internal_t;

#define REVISION2 0xa609
typedef struct {
	uint8_t fpga_minor_revision: 8;
} revision2_internal_t;

#define ACCEL_X_FRE 0xa60a
typedef struct {
	uint8_t accel_x_fre: 8;
} accel_x_fre_internal_t;

#define ACCEL_X_AMP 0xa60b
typedef struct {
	uint8_t accel_x_amp: 8;
} accel_x_amp_internal_t;

#define ACCEL_X_OFS 0xa60c
typedef struct {
	uint8_t accel_x_ofs: 8;
} accel_x_ofs_internal_t;

#define ACCEL_X_PHA 0xa60d
typedef struct {
	uint8_t accel_x_dith_en: 1;
	uint8_t accel_x_pha: 7;
} accel_x_pha_internal_t;

#define ACCEL_X_Y_AEX 0xa60e
typedef struct {
	uint8_t accel_y_aex: 4;
	uint8_t accel_x_aex: 4;
} accel_x_y_aex_internal_t;

#define ACCEL_Y_FRE 0xa60f
typedef struct {
	uint8_t accel_y_fre: 8;
} accel_y_fre_internal_t;

#define ACCEL_Y_AMP 0xa610
typedef struct {
	uint8_t accel_y_amp: 8;
} accel_y_amp_internal_t;

#define ACCEL_Y_OFS 0xa611
typedef struct {
	uint8_t accel_y_ofs: 8;
} accel_y_ofs_internal_t;

#define ACCEL_Y_PHA 0xa612
typedef struct {
	uint8_t accel_y_dith_en: 1;
	uint8_t accel_y_pha: 7;
} accel_y_pha_internal_t;

#define ACCEL_Z_FRE 0xa613
typedef struct {
	uint8_t accel_z_fre: 8;
} accel_z_fre_internal_t;

#define ACCEL_Z_AMP 0xa614
typedef struct {
	uint8_t accel_z_amp: 8;
} accel_z_amp_internal_t;

#define ACCEL_Z_OFS 0xa615
typedef struct {
	uint8_t accel_z_ofs: 8;
} accel_z_ofs_internal_t;

#define ACCEL_Z_PHA 0xa616
typedef struct {
	uint8_t accel_z_dith_en: 1;
	uint8_t accel_z_pha: 7;
} accel_z_pha_internal_t;

#define ACCEL_Z_GYRO_X_AEX 0xa617
typedef struct {
	uint8_t gyro_x_aex: 4;
	uint8_t accel_z_aex: 4;
} accel_z_gyro_x_aex_internal_t;

#define GYRO_X_FRE 0xa618
typedef struct {
	uint8_t gyro_x_fre: 8;
} gyro_x_fre_internal_t;

#define GYRO_X_AMP 0xa619
typedef struct {
	uint8_t gyro_x_amp: 8;
} gyro_x_amp_internal_t;

#define GYRO_X_OFS 0xa61a
typedef struct {
	uint8_t gyro_x_ofs: 8;
} gyro_x_ofs_internal_t;

#define GYRO_X_PHA 0xa61b
typedef struct {
	uint8_t gyro_x_dith_en: 1;
	uint8_t gyro_x_pha: 7;
} gyro_x_pha_internal_t;

#define GYRO_Y_FRE 0xa61c
typedef struct {
	uint8_t gyro_y_fre: 8;
} gyro_y_fre_internal_t;

#define GYRO_Y_AMP 0xa61d
typedef struct {
	uint8_t gyro_y_amp: 8;
} gyro_y_amp_internal_t;

#define GYRO_Y_OFS 0xa61e
typedef struct {
	uint8_t gyro_y_ofs: 8;
} gyro_y_ofs_internal_t;

#define GYRO_Y_PHA 0xa61f
typedef struct {
	uint8_t gyro_y_dith_en: 1;
	uint8_t gyro_y_pha: 7;
} gyro_y_pha_internal_t;

#define GYRO_Y_GYRO_Z_AEX 0xa620
typedef struct {
	uint8_t gyro_z_aex: 4;
	uint8_t gyro_y_aex: 4;
} gyro_y_gyro_z_aex_internal_t;

#define GYRO_Z_FRE 0xa621
typedef struct {
	uint8_t gyro_z_fre: 8;
} gyro_z_fre_internal_t;

#define GYRO_Z_AMP 0xa622
typedef struct {
	uint8_t gyro_z_amp: 8;
} gyro_z_amp_internal_t;

#define GYRO_Z_OFS 0xa623
typedef struct {
	uint8_t gyro_z_ofs: 8;
} gyro_z_ofs_internal_t;

#define GYRO_Z_PHA 0xa624
typedef struct {
	uint8_t gyro_z_dith_en: 1;
	uint8_t gyro_z_pha: 7;
} gyro_z_pha_internal_t;

#define ACCEL_GYRO_CLKDIV 0xa625
typedef struct {
	uint8_t gyro_clk_div: 4;
	uint8_t accel_clk_div: 4;
} accel_gyro_clkdiv_internal_t;

#define FPGA_CTRL 0xa627
typedef struct {
	uint8_t bram_wr_dis: 1;
	uint8_t ram_ovf: 1;
	uint8_t resv_1: 6;
} fpga_ctrl_internal_t;

#define OTP_ERROR_STATUS 0xa628
typedef struct {
	uint8_t diff_read_mismatch_error: 1;
	uint8_t prog_more_than_1_bit_error: 1;
	uint8_t prog_0_bit_error: 1;
	uint8_t cfg_mismatch_error: 1;
	uint8_t resv_1: 4;
} otp_error_status_internal_t;

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
} fpga_reg0_internal_t;

#define FPGA_REG1 0xa637
typedef struct {
	uint8_t fpga_dbg1: 8;
} fpga_reg1_internal_t;

#define FPGA_REG2 0xa638
typedef struct {
	uint8_t fpga_dbg2: 8;
} fpga_reg2_internal_t;

#define FPGA_REG3 0xa639
typedef struct {
	uint8_t fpga_dbg3: 8;
} fpga_reg3_internal_t;

#define FPGA_REG4 0xa63a
typedef struct {
	uint8_t fpga_dbg4: 8;
} fpga_reg4_internal_t;

#define FPGA_REG5 0xa63b
typedef struct {
	uint8_t fpga_dbg5: 8;
} fpga_reg5_internal_t;

#define FPGA_STAT0 0xa63c
typedef struct {
	uint8_t fpga_dbgstat0: 8;
} fpga_stat0_internal_t;

#define FPGA_STAT1 0xa63d
typedef struct {
	uint8_t fpga_dbgstat1: 8;
} fpga_stat1_internal_t;

/* IPREG_OTP */
#define OTP_MRA_0 0xaf00
#define OTP_MRA_1 0xaf01
#define OTP_MRB_0 0xaf02
#define OTP_MRB_1 0xaf03
#define OTP_MR_0  0xaf04
#define OTP_MR_1  0xaf05
#define OTP_CTRL  0xaf06
typedef struct {
	uint8_t otp_soak: 1;
	uint8_t otp_pwr_down: 1;
	uint8_t otp_low_pwr_mode: 1;
	uint8_t otp_reload: 1;
	uint8_t otp_boot: 1;
	uint8_t otp_stress: 1;
	uint8_t otp_reset: 1;
	uint8_t pgm_dly_cfg: 1;
} otp_ctrl_internal_t;

/* IPREG_SYS1 */
#define IPREG_SYS1_REG_0   0xa400
#define IPREG_SYS1_REG_1   0xa401
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
#define IPREG_SYS1_REG_14  0xa40e
#define IPREG_SYS1_REG_15  0xa40f
#define IPREG_SYS1_REG_16  0xa410
#define IPREG_SYS1_REG_17  0xa411
#define IPREG_SYS1_REG_18  0xa412
#define IPREG_SYS1_REG_19  0xa413
#define IPREG_SYS1_REG_20  0xa414
#define IPREG_SYS1_REG_21  0xa415
#define IPREG_SYS1_REG_22  0xa416
#define IPREG_SYS1_REG_23  0xa417
#define IPREG_SYS1_REG_24  0xa418
#define IPREG_SYS1_REG_25  0xa419
#define IPREG_SYS1_REG_26  0xa41a
#define IPREG_SYS1_REG_27  0xa41b
#define IPREG_SYS1_REG_28  0xa41c
#define IPREG_SYS1_REG_29  0xa41d
#define IPREG_SYS1_REG_30  0xa41e
#define IPREG_SYS1_REG_31  0xa41f
#define IPREG_SYS1_REG_32  0xa420
#define IPREG_SYS1_REG_33  0xa421
#define IPREG_SYS1_REG_34  0xa422
#define IPREG_SYS1_REG_35  0xa423
#define IPREG_SYS1_REG_36  0xa424
#define IPREG_SYS1_REG_37  0xa425
#define IPREG_SYS1_REG_38  0xa426
#define IPREG_SYS1_REG_39  0xa427
#define IPREG_SYS1_REG_40  0xa428
#define IPREG_SYS1_REG_41  0xa429
#define IPREG_SYS1_REG_42  0xa42a
#define IPREG_SYS1_REG_43  0xa42b
#define IPREG_SYS1_REG_44  0xa42c
#define IPREG_SYS1_REG_45  0xa42d
#define IPREG_SYS1_REG_46  0xa42e
#define IPREG_SYS1_REG_47  0xa42f
#define IPREG_SYS1_REG_48  0xa430
#define IPREG_SYS1_REG_49  0xa431
#define IPREG_SYS1_REG_50  0xa432
#define IPREG_SYS1_REG_51  0xa433
#define IPREG_SYS1_REG_52  0xa434
#define IPREG_SYS1_REG_53  0xa435
#define IPREG_SYS1_REG_54  0xa436
#define IPREG_SYS1_REG_55  0xa437
#define IPREG_SYS1_REG_56  0xa438
#define IPREG_SYS1_REG_57  0xa439
#define IPREG_SYS1_REG_58  0xa43a
#define IPREG_SYS1_REG_59  0xa43b
#define IPREG_SYS1_REG_60  0xa43c
#define IPREG_SYS1_REG_61  0xa43d
#define IPREG_SYS1_REG_62  0xa43e
#define IPREG_SYS1_REG_63  0xa43f
#define IPREG_SYS1_REG_64  0xa440
#define IPREG_SYS1_REG_65  0xa441
#define IPREG_SYS1_REG_66  0xa442
#define IPREG_SYS1_REG_67  0xa443
#define IPREG_SYS1_REG_68  0xa444
#define IPREG_SYS1_REG_69  0xa445
#define IPREG_SYS1_REG_70  0xa446
#define IPREG_SYS1_REG_71  0xa447
#define IPREG_SYS1_REG_72  0xa448
#define IPREG_SYS1_REG_73  0xa449
#define IPREG_SYS1_REG_74  0xa44a
#define IPREG_SYS1_REG_75  0xa44b
#define IPREG_SYS1_REG_76  0xa44c
#define IPREG_SYS1_REG_77  0xa44d
#define IPREG_SYS1_REG_78  0xa44e
#define IPREG_SYS1_REG_79  0xa44f
#define IPREG_SYS1_REG_80  0xa450
#define IPREG_SYS1_REG_81  0xa451
#define IPREG_SYS1_REG_82  0xa452
#define IPREG_SYS1_REG_83  0xa453
#define IPREG_SYS1_REG_84  0xa454
#define IPREG_SYS1_REG_85  0xa455
#define IPREG_SYS1_REG_86  0xa456
#define IPREG_SYS1_REG_87  0xa457
#define IPREG_SYS1_REG_88  0xa458
#define IPREG_SYS1_REG_89  0xa459
#define IPREG_SYS1_REG_90  0xa45a
#define IPREG_SYS1_REG_91  0xa45b
#define IPREG_SYS1_REG_92  0xa45c
#define IPREG_SYS1_REG_93  0xa45d
#define IPREG_SYS1_REG_94  0xa45e
#define IPREG_SYS1_REG_95  0xa45f
#define IPREG_SYS1_REG_96  0xa460
#define IPREG_SYS1_REG_97  0xa461
#define IPREG_SYS1_REG_98  0xa462
#define IPREG_SYS1_REG_99  0xa463
#define IPREG_SYS1_REG_100 0xa464
#define IPREG_SYS1_REG_101 0xa465
#define IPREG_SYS1_REG_102 0xa466
#define IPREG_SYS1_REG_103 0xa467
#define IPREG_SYS1_REG_104 0xa468
#define IPREG_SYS1_REG_105 0xa469
#define IPREG_SYS1_REG_106 0xa46a
#define IPREG_SYS1_REG_107 0xa46b
#define IPREG_SYS1_REG_108 0xa46c
#define IPREG_SYS1_REG_109 0xa46d
#define IPREG_SYS1_REG_110 0xa46e
#define IPREG_SYS1_REG_111 0xa46f
#define IPREG_SYS1_REG_112 0xa470
#define IPREG_SYS1_REG_113 0xa471
#define IPREG_SYS1_REG_114 0xa472
#define IPREG_SYS1_REG_115 0xa473
#define IPREG_SYS1_REG_116 0xa474
#define IPREG_SYS1_REG_117 0xa475
#define IPREG_SYS1_REG_118 0xa476
#define IPREG_SYS1_REG_119 0xa477
#define IPREG_SYS1_REG_120 0xa478
#define IPREG_SYS1_REG_121 0xa479
#define IPREG_SYS1_REG_122 0xa47a
#define IPREG_SYS1_REG_123 0xa47b
#define IPREG_SYS1_REG_124 0xa47c
#define IPREG_SYS1_REG_125 0xa47d
#define IPREG_SYS1_REG_126 0xa47e
#define IPREG_SYS1_REG_127 0xa47f
#define IPREG_SYS1_REG_128 0xa480
typedef struct {
	uint8_t gyro_x_gaintca_hfs: 8;
} ipreg_sys1_reg_128_internal_t;

#define IPREG_SYS1_REG_129 0xa481
typedef struct {
	uint8_t gyro_x_gaintca_lfs: 8;
} ipreg_sys1_reg_129_internal_t;

#define IPREG_SYS1_REG_130 0xa482
typedef struct {
	uint8_t gyro_x_gaintcb_hfs: 8;
} ipreg_sys1_reg_130_internal_t;

#define IPREG_SYS1_REG_131 0xa483
typedef struct {
	uint8_t gyro_x_gaintcb_lfs: 8;
} ipreg_sys1_reg_131_internal_t;

#define IPREG_SYS1_REG_132 0xa484
typedef struct {
	uint8_t gyro_y_gaintca_hfs: 8;
} ipreg_sys1_reg_132_internal_t;

#define IPREG_SYS1_REG_133 0xa485
typedef struct {
	uint8_t gyro_y_gaintca_lfs: 8;
} ipreg_sys1_reg_133_internal_t;

#define IPREG_SYS1_REG_134 0xa486
typedef struct {
	uint8_t gyro_y_gaintcb_hfs: 8;
} ipreg_sys1_reg_134_internal_t;

#define IPREG_SYS1_REG_135 0xa487
typedef struct {
	uint8_t gyro_y_gaintcb_lfs: 8;
} ipreg_sys1_reg_135_internal_t;

#define IPREG_SYS1_REG_136 0xa488
typedef struct {
	uint8_t gyro_z_gaintca_hfs: 8;
} ipreg_sys1_reg_136_internal_t;

#define IPREG_SYS1_REG_137 0xa489
typedef struct {
	uint8_t gyro_z_gaintca_lfs: 8;
} ipreg_sys1_reg_137_internal_t;

#define IPREG_SYS1_REG_138 0xa48a
typedef struct {
	uint8_t gyro_z_gaintcb_hfs: 8;
} ipreg_sys1_reg_138_internal_t;

#define IPREG_SYS1_REG_139 0xa48b
typedef struct {
	uint8_t gyro_z_gaintcb_lfs: 8;
} ipreg_sys1_reg_139_internal_t;

#define IPREG_SYS1_REG_140 0xa48c
typedef struct {
	uint8_t gyro_x_notch_pref_sel: 2;
	uint8_t gyro_x_notch1_sel: 5;
	uint8_t gyro_x_notch2_sel: 1;
} ipreg_sys1_reg_140_internal_t;

#define IPREG_SYS1_REG_141 0xa48d
#define IPREG_SYS1_REG_142 0xa48e
#define IPREG_SYS1_REG_143 0xa48f
#define IPREG_SYS1_REG_144 0xa490
#define IPREG_SYS1_REG_145 0xa491
typedef struct {
	uint8_t gyro_x_afsr_th_high: 4;
	uint8_t gyro_x_afsr_th_low: 4;
} ipreg_sys1_reg_145_internal_t;

#define IPREG_SYS1_REG_146 0xa492
typedef struct {
	uint8_t gyro_x_tmid_gain: 4;
	uint8_t gyro_x_tmid_off: 4;
} ipreg_sys1_reg_146_internal_t;

#define IPREG_SYS1_REG_147 0xa493
typedef struct {
	uint8_t gyro_y_afsr_th_high: 4;
	uint8_t gyro_y_afsr_th_low: 4;
} ipreg_sys1_reg_147_internal_t;

#define IPREG_SYS1_REG_148 0xa494
typedef struct {
	uint8_t gyro_y_tmid_gain: 4;
	uint8_t gyro_y_tmid_off: 4;
} ipreg_sys1_reg_148_internal_t;

#define IPREG_SYS1_REG_149 0xa495
typedef struct {
	uint8_t gyro_z_afsr_th_high: 4;
	uint8_t gyro_z_afsr_th_low: 4;
} ipreg_sys1_reg_149_internal_t;

#define IPREG_SYS1_REG_150 0xa496
typedef struct {
	uint8_t gyro_z_tmid_gain: 4;
	uint8_t gyro_z_tmid_off: 4;
} ipreg_sys1_reg_150_internal_t;

#define IPREG_SYS1_REG_151 0xa497
typedef struct {
	uint8_t gyro_temp_odr: 2;
	uint8_t gyro_x_afsr_pulse_qual: 2;
	uint8_t gyro_y_afsr_pulse_qual: 2;
	uint8_t gyro_z_afsr_pulse_qual: 2;
} ipreg_sys1_reg_151_internal_t;

#define IPREG_SYS1_REG_152 0xa498
#define IPREG_SYS1_REG_153 0xa499
#define IPREG_SYS1_REG_154 0xa49a
typedef struct {
	uint8_t gyro_afsr_enable: 1;
	uint8_t gyro_fsr: 1;
	uint8_t gyro_src_ctrl: 2;
	uint8_t gyro_afsr_shared: 1;
	uint8_t gyro_ois_m6_byp: 1;
	uint8_t resv_1: 2;
} ipreg_sys1_reg_154_internal_t;

#define IPREG_SYS1_REG_155 0xa49b
typedef struct {
	uint8_t gyro_ois_hpf_byp: 1;
	uint8_t gyro_lpf_bypass: 1;
	uint8_t resv_1: 6;
} ipreg_sys1_reg_155_internal_t;

#define IPREG_SYS1_REG_156 0xa49c
typedef struct {
	uint8_t gyro_dec1_sat_det: 3;
	uint8_t resv_1: 5;
} ipreg_sys1_reg_156_internal_t;

#define IPREG_SYS1_REG_157 0xa49d
typedef struct {
	uint8_t gyro_lp_avg_sel: 4;
	uint8_t gyro_ois_hpfbw_sel: 3;
	uint8_t gyro_notch_bypass: 1;
} ipreg_sys1_reg_157_internal_t;

#define IPREG_SYS1_REG_158 0xa49e
typedef struct {
	uint8_t gyro_ois_lpfbw_sel: 3;
	uint8_t gyro_ois_3rd_ord_sel: 1;
	uint8_t gyro_ui_lpfbw_sel: 3;
	uint8_t gyro_ui_3rd_ord_sel: 1;
} ipreg_sys1_reg_158_internal_t;

#define IPREG_SYS1_REG_159 0xa49f
typedef struct {
	uint8_t sigp_pld_disable: 1;
	uint8_t gyro_inj_en_gos: 1;
	uint8_t gyro_observe_adc: 1;
	uint8_t gyro_observe_dec1: 1;
	uint8_t resv_1: 4;
} ipreg_sys1_reg_159_internal_t;

#define IPREG_SYS1_REG_160 0xa4a0
typedef struct {
	uint8_t gyro_lfs_value: 2;
	uint8_t gyro_dec1_freeze_trim: 3;
	uint8_t gyro_indep_ana_fs: 1;
	uint8_t resv_1: 2;
} ipreg_sys1_reg_160_internal_t;

/* IPREG_SYS2 */
#define IPREG_SYS2_REG_0  0xa500
#define IPREG_SYS2_REG_1  0xa501
#define IPREG_SYS2_REG_2  0xa502
#define IPREG_SYS2_REG_3  0xa503
#define IPREG_SYS2_REG_4  0xa504
#define IPREG_SYS2_REG_5  0xa505
#define IPREG_SYS2_REG_6  0xa506
#define IPREG_SYS2_REG_7  0xa507
#define IPREG_SYS2_REG_8  0xa508
#define IPREG_SYS2_REG_9  0xa509
#define IPREG_SYS2_REG_10 0xa50a
#define IPREG_SYS2_REG_11 0xa50b
#define IPREG_SYS2_REG_12 0xa50c
#define IPREG_SYS2_REG_13 0xa50d
#define IPREG_SYS2_REG_14 0xa50e
#define IPREG_SYS2_REG_15 0xa50f
#define IPREG_SYS2_REG_16 0xa510
#define IPREG_SYS2_REG_17 0xa511
#define IPREG_SYS2_REG_18 0xa512
#define IPREG_SYS2_REG_19 0xa513
#define IPREG_SYS2_REG_20 0xa514
#define IPREG_SYS2_REG_21 0xa515
#define IPREG_SYS2_REG_22 0xa516
#define IPREG_SYS2_REG_23 0xa517
#define IPREG_SYS2_REG_24 0xa518
#define IPREG_SYS2_REG_25 0xa519
#define IPREG_SYS2_REG_26 0xa51a
#define IPREG_SYS2_REG_27 0xa51b
#define IPREG_SYS2_REG_28 0xa51c
#define IPREG_SYS2_REG_29 0xa51d
#define IPREG_SYS2_REG_30 0xa51e
#define IPREG_SYS2_REG_31 0xa51f
#define IPREG_SYS2_REG_32 0xa520
#define IPREG_SYS2_REG_33 0xa521
#define IPREG_SYS2_REG_34 0xa522
#define IPREG_SYS2_REG_35 0xa523
#define IPREG_SYS2_REG_36 0xa524
#define IPREG_SYS2_REG_37 0xa525
#define IPREG_SYS2_REG_38 0xa526
#define IPREG_SYS2_REG_39 0xa527
#define IPREG_SYS2_REG_40 0xa528
#define IPREG_SYS2_REG_41 0xa529
#define IPREG_SYS2_REG_42 0xa52a
#define IPREG_SYS2_REG_43 0xa52b
#define IPREG_SYS2_REG_44 0xa52c
#define IPREG_SYS2_REG_45 0xa52d
#define IPREG_SYS2_REG_46 0xa52e
#define IPREG_SYS2_REG_47 0xa52f
#define IPREG_SYS2_REG_48 0xa530
#define IPREG_SYS2_REG_49 0xa531
#define IPREG_SYS2_REG_50 0xa532
#define IPREG_SYS2_REG_51 0xa533
#define IPREG_SYS2_REG_52 0xa534
#define IPREG_SYS2_REG_53 0xa535
#define IPREG_SYS2_REG_54 0xa536
#define IPREG_SYS2_REG_55 0xa537
#define IPREG_SYS2_REG_56 0xa538
#define IPREG_SYS2_REG_57 0xa539
#define IPREG_SYS2_REG_58 0xa53a
#define IPREG_SYS2_REG_59 0xa53b
#define IPREG_SYS2_REG_60 0xa53c
#define IPREG_SYS2_REG_61 0xa53d
#define IPREG_SYS2_REG_62 0xa53e
#define IPREG_SYS2_REG_63 0xa53f
#define IPREG_SYS2_REG_64 0xa540
#define IPREG_SYS2_REG_65 0xa541
#define IPREG_SYS2_REG_66 0xa542
#define IPREG_SYS2_REG_67 0xa543
#define IPREG_SYS2_REG_68 0xa544
#define IPREG_SYS2_REG_69 0xa545
#define IPREG_SYS2_REG_70 0xa546
#define IPREG_SYS2_REG_71 0xa547
#define IPREG_SYS2_REG_72 0xa548
#define IPREG_SYS2_REG_73 0xa549
#define IPREG_SYS2_REG_74 0xa54a
#define IPREG_SYS2_REG_75 0xa54b
#define IPREG_SYS2_REG_76 0xa54c
#define IPREG_SYS2_REG_77 0xa54d
#define IPREG_SYS2_REG_78 0xa54e
#define IPREG_SYS2_REG_79 0xa54f
#define IPREG_SYS2_REG_80 0xa550
#define IPREG_SYS2_REG_81 0xa551
#define IPREG_SYS2_REG_82 0xa552
#define IPREG_SYS2_REG_83 0xa553
#define IPREG_SYS2_REG_84 0xa554
#define IPREG_SYS2_REG_85 0xa555
#define IPREG_SYS2_REG_86 0xa556
#define IPREG_SYS2_REG_87 0xa557
typedef struct {
	uint8_t accel_x_gaintca_hfs: 8;
} ipreg_sys2_reg_87_internal_t;

#define IPREG_SYS2_REG_88 0xa558
typedef struct {
	uint8_t accel_x_gaintca_hfs32: 8;
} ipreg_sys2_reg_88_internal_t;

#define IPREG_SYS2_REG_89 0xa559
typedef struct {
	uint8_t accel_x_gaintca_lfs: 8;
} ipreg_sys2_reg_89_internal_t;

#define IPREG_SYS2_REG_90 0xa55a
typedef struct {
	uint8_t accel_x_gaintcb_hfs: 8;
} ipreg_sys2_reg_90_internal_t;

#define IPREG_SYS2_REG_91 0xa55b
typedef struct {
	uint8_t accel_x_gaintcb_hfs32: 8;
} ipreg_sys2_reg_91_internal_t;

#define IPREG_SYS2_REG_92 0xa55c
typedef struct {
	uint8_t accel_x_gaintcb_lfs: 8;
} ipreg_sys2_reg_92_internal_t;

#define IPREG_SYS2_REG_93 0xa55d
typedef struct {
	uint8_t accel_y_gaintca_hfs: 8;
} ipreg_sys2_reg_93_internal_t;

#define IPREG_SYS2_REG_94 0xa55e
typedef struct {
	uint8_t accel_y_gaintca_hfs32: 8;
} ipreg_sys2_reg_94_internal_t;

#define IPREG_SYS2_REG_95 0xa55f
typedef struct {
	uint8_t accel_y_gaintca_lfs: 8;
} ipreg_sys2_reg_95_internal_t;

#define IPREG_SYS2_REG_96 0xa560
typedef struct {
	uint8_t accel_y_gaintcb_hfs: 8;
} ipreg_sys2_reg_96_internal_t;

#define IPREG_SYS2_REG_97 0xa561
typedef struct {
	uint8_t accel_y_gaintcb_hfs32: 8;
} ipreg_sys2_reg_97_internal_t;

#define IPREG_SYS2_REG_98 0xa562
typedef struct {
	uint8_t accel_y_gaintcb_lfs: 8;
} ipreg_sys2_reg_98_internal_t;

#define IPREG_SYS2_REG_99 0xa563
typedef struct {
	uint8_t accel_z_gaintca_hfs: 8;
} ipreg_sys2_reg_99_internal_t;

#define IPREG_SYS2_REG_100 0xa564
typedef struct {
	uint8_t accel_z_gaintca_hfs32: 8;
} ipreg_sys2_reg_100_internal_t;

#define IPREG_SYS2_REG_101 0xa565
typedef struct {
	uint8_t accel_z_gaintca_lfs: 8;
} ipreg_sys2_reg_101_internal_t;

#define IPREG_SYS2_REG_102 0xa566
typedef struct {
	uint8_t accel_z_gaintcb_hfs: 8;
} ipreg_sys2_reg_102_internal_t;

#define IPREG_SYS2_REG_103 0xa567
typedef struct {
	uint8_t accel_z_gaintcb_hfs32: 8;
} ipreg_sys2_reg_103_internal_t;

#define IPREG_SYS2_REG_104 0xa568
typedef struct {
	uint8_t accel_z_gaintcb_lfs: 8;
} ipreg_sys2_reg_104_internal_t;

#define IPREG_SYS2_REG_105 0xa569
typedef struct {
	uint8_t accel_x_tmid_gain: 4;
	uint8_t accel_x_tmid_off: 4;
} ipreg_sys2_reg_105_internal_t;

#define IPREG_SYS2_REG_106 0xa56a
typedef struct {
	uint8_t accel_y_tmid_gain: 4;
	uint8_t accel_y_tmid_off: 4;
} ipreg_sys2_reg_106_internal_t;

#define IPREG_SYS2_REG_107 0xa56b
typedef struct {
	uint8_t accel_z_tmid_gain: 4;
	uint8_t accel_z_tmid_off: 4;
} ipreg_sys2_reg_107_internal_t;

#define IPREG_SYS2_REG_108 0xa56c
typedef struct {
	uint8_t accel_temp_odr: 3;
	uint8_t accel_32gee_fs: 1;
	uint8_t accel_dec1_freeze_trim: 3;
	uint8_t resv_1: 1;
} ipreg_sys2_reg_108_internal_t;

#define IPREG_SYS2_REG_109 0xa56d
typedef struct {
	uint8_t accel_src_ctrl: 2;
	uint8_t resv_1: 6;
} ipreg_sys2_reg_109_internal_t;

#define IPREG_SYS2_REG_110 0xa56e
typedef struct {
	uint8_t accel_lp_avg_sel: 4;
	uint8_t accel_ois_hpfbw_sel: 3;
	uint8_t resv_1: 1;
} ipreg_sys2_reg_110_internal_t;

#define IPREG_SYS2_REG_111 0xa56f
typedef struct {
	uint8_t accel_ois_lpfbw_sel: 3;
	uint8_t accel_ois_3rd_ord_sel: 1;
	uint8_t resv_1: 4;
} ipreg_sys2_reg_111_internal_t;

#define IPREG_SYS2_REG_112 0xa570
typedef struct {
	uint8_t accel_ui_lpfbw_sel: 3;
	uint8_t accel_ui_3rd_ord_sel: 1;
	uint8_t accel_lpf_bypass: 1;
	uint8_t accel_ois_hpf_byp: 1;
	uint8_t accel_ois_m6_byp: 1;
	uint8_t resv_1: 1;
} ipreg_sys2_reg_112_internal_t;

#define IPREG_SYS2_REG_113 0xa571
typedef struct {
	uint8_t accel_observe_adc: 1;
	uint8_t accel_observe_dec1: 1;
	uint8_t accel_inj_en_adc: 1;
	uint8_t accel_inj_en_gos: 1;
	uint8_t accel_tst_dig_integ_dis: 2;
	uint8_t resv_1: 2;
} ipreg_sys2_reg_113_internal_t;

#define IPREG_SYS2_REG_114 0xa572
typedef struct {
	uint8_t tmp_co_trim: 7;
	uint8_t resv_1: 1;
} ipreg_sys2_reg_114_internal_t;

#define IPREG_SYS2_REG_115 0xa573
typedef struct {
	uint8_t tmp_offset_trim: 7;
	uint8_t resv_1: 1;
} ipreg_sys2_reg_115_internal_t;

#define IPREG_SYS2_REG_116 0xa574
typedef struct {
	uint8_t tmp_gain_trim: 4;
	uint8_t tmp_dis: 1;
	uint8_t resv_1: 3;
} ipreg_sys2_reg_116_internal_t;

#define IPREG_SYS2_REG_117 0xa575
typedef struct {
	uint8_t tmp_inj_en_gos: 1;
	uint8_t tmp_lpf_cfg: 3;
	uint8_t tmp_dec_cfg: 3;
	uint8_t resv_1: 1;
} ipreg_sys2_reg_117_internal_t;

#define IPREG_SYS2_REG_118 0xa576
typedef struct {
	uint8_t accel_indep_ana_fs: 1;
	uint8_t accel_fsr: 2;
	uint8_t resv_1: 5;
} ipreg_sys2_reg_118_internal_t;

/* IPREG_TOP1 */
#define FIFO_STATUS 0xa200
typedef struct {
	uint8_t fifo_insufficient_mem: 1;
	uint8_t fifo_overflow_state: 1;
	uint8_t resv_1: 6;
} fifo_status_internal_t;

#define PRG_ERR_STATUS 0xa202
typedef struct {
	uint8_t prog_err_status: 1;
	uint8_t resv_1: 7;
} prg_err_status_internal_t;

#define FIFO_LOST_COUNT 0xa204
typedef struct {
	uint8_t fifo_lost_frame_cnt: 8;
} fifo_lost_count_internal_t;

#define FIFO_SRAM_OVRD 0xa205
typedef struct {
	uint8_t fifo_gsleep_sram_override_actv: 1;
	uint8_t fifo_gsleep_sram_override_inactv: 1;
	uint8_t fifo_retention_sram_override_actv: 1;
	uint8_t fifo_retention_sram_override_inactv: 1;
	uint8_t resv_1: 4;
} fifo_sram_ovrd_internal_t;

#define DEBUG_CONTROL 0xa23d
typedef struct {
	uint8_t debg_1: 8;
} debug_control_internal_t;

#define SRAM_ROM_MARGIN_CTRL 0xa23e
typedef struct {
	uint8_t fifo_sram_margin: 2;
	uint8_t rom_margin: 1;
	uint8_t resv_1: 5;
} sram_rom_margin_ctrl_internal_t;

#define I3C_PROV_ID_0 0xa23f
#define I3C_PROV_ID_1 0xa240
#define I3C_PROV_ID_2 0xa241
#define I3C_PROV_ID_3 0xa242
#define I3C_PROV_ID_4 0xa243
#define I3C_PROV_ID_5 0xa244
#define SIFS_I3C_BCR  0xa245
typedef struct {
	uint8_t i3c_bcr: 8;
} sifs_i3c_bcr_internal_t;

#define SIFS_I3C_DCR 0xa246
typedef struct {
	uint8_t i3c_dcr: 8;
} sifs_i3c_dcr_internal_t;

#define SIFS_CONTROL 0xa247
typedef struct {
	uint8_t resv_1: 1;
	uint8_t hot_join_en: 1;
	uint8_t i3c_sdr_en: 1;
	uint8_t i3c_ddr_en: 1;
	uint8_t resv_2: 4;
} sifs_control_internal_t;

#define SIFS_GETMXDS_MAXWR 0xa248
typedef struct {
	uint8_t getmxds_maxwr: 3;
	uint8_t resv_1: 5;
} sifs_getmxds_maxwr_internal_t;

#define SIFS_GETMXDS_MAXRD 0xa249
typedef struct {
	uint8_t getmxds_maxrd: 6;
	uint8_t resv_1: 2;
} sifs_getmxds_maxrd_internal_t;

#define SIFS_IXC_ERROR_STATUS 0xa24b
typedef struct {
	uint8_t sifs_ixc_timeout_err: 1;
	uint8_t resv_1: 7;
} sifs_ixc_error_status_internal_t;

#define INTF_CONFIG 0xa24e
typedef struct {
	uint8_t i2c_sda_out_dly: 4;
	uint8_t i3c_sda_out_dly: 3;
	uint8_t resv_1: 1;
} intf_config_internal_t;

#define EDMP_PRGRM_IRQ0_0  0xa24f
#define EDMP_PRGRM_IRQ0_1  0xa250
#define EDMP_PRGRM_IRQ1_0  0xa251
#define EDMP_PRGRM_IRQ1_1  0xa252
#define EDMP_PRGRM_IRQ2_0  0xa253
#define EDMP_PRGRM_IRQ2_1  0xa254
#define EDMP_SP_START_ADDR 0xa255
typedef struct {
	uint8_t sp_start_addr: 8;
} edmp_sp_start_addr_internal_t;

#define PMC_AGC_TIMER_CFG 0xa256
typedef struct {
	uint8_t resv_1: 1;
	uint8_t sense_en_del_trim: 3;
	uint8_t gyro_drive_ovd_ctrl_trim: 1;
	uint8_t agc_timer_cfg: 2;
	uint8_t force_gyro_sns_en: 1;
} pmc_agc_timer_cfg_internal_t;

#define PMC_AGC_TIMER_CFG_0 0xa257
typedef struct {
	uint8_t gyro_wdog_dis: 1;
	uint8_t resv_1: 7;
} pmc_agc_timer_cfg_0_internal_t;

#define SMC_CONTROL_0 0xa258
typedef struct {
	uint8_t tmst_en: 1;
	uint8_t tmst_fsync_en: 1;
	uint8_t tmst_force_aux_fine_en: 1;
	uint8_t temp_dis: 1;
	uint8_t resv_1: 1;
	uint8_t accel_ap_en: 1;
	uint8_t gyro_ap_en: 1;
	uint8_t shared_cnt_dis: 1;
} smc_control_0_internal_t;

#define SMC_CONTROL_1 0xa259
typedef struct {
	uint8_t ulp_ed_rcosc_wu_trim: 3;
	uint8_t sreg_aux_accel_only_en: 1;
	uint8_t resv_1: 4;
} smc_control_1_internal_t;

#define SMC_ANA_LAT_TRIM_0 0xa25a
typedef struct {
	uint8_t accel_ana_lat_trim: 5;
	uint8_t resv_1: 3;
} smc_ana_lat_trim_0_internal_t;

#define SMC_ANA_LAT_TRIM_1 0xa25b
typedef struct {
	uint8_t gyro_ana_lat_trim: 5;
	uint8_t resv_1: 3;
} smc_ana_lat_trim_1_internal_t;

#define SMC_ANA_LAT_TRIM_STC 0xa25c
typedef struct {
	uint8_t accel_ana_lat_trim_stc_mode: 5;
	uint8_t resv_1: 3;
} smc_ana_lat_trim_stc_internal_t;

#define SMC_ANA_LAT_TRIM_STC_0 0xa25d
typedef struct {
	uint8_t gyro_ana_lat_trim_stc_mode: 5;
	uint8_t resv_1: 3;
} smc_ana_lat_trim_stc_0_internal_t;

#define SMC_DEC1_TON 0xa25e
typedef struct {
	uint8_t accel_dec1_ton: 3;
	uint8_t gyro_dec1_ton: 3;
	uint8_t resv_1: 2;
} smc_dec1_ton_internal_t;

#define SMC_ODR_STAT 0xa25f
typedef struct {
	uint8_t accel_ap_status_odr: 4;
	uint8_t gyro_ap_status_odr: 4;
} smc_odr_stat_internal_t;

#define SREG_CTRL 0xa260
typedef struct {
	uint8_t sreg_invalid_ind_dis: 1;
	uint8_t sreg_data_endian_sel: 1;
	uint8_t sreg_debug_data_inject_en: 1;
	uint8_t sreg_sifs_20bits_en: 1;
	uint8_t resv_1: 4;
} sreg_ctrl_internal_t;

#define SIFS_I3C_STC_CFG 0xa261
typedef struct {
	uint8_t i3c_ibi_byte_en: 1;
	uint8_t i3c_ibi_en: 1;
	uint8_t asynctime0_dis: 1;
	uint8_t reset_whole_target_dis: 1;
	uint8_t resv_1: 4;
} sifs_i3c_stc_cfg_internal_t;

#define INT_PULSE_MIN_ON_INTF0 0xa262
typedef struct {
	uint8_t int0_tpulse_duration: 3;
	uint8_t resv_1: 5;
} int_pulse_min_on_intf0_internal_t;

#define INT_PULSE_MIN_ON_INTF1 0xa263
typedef struct {
	uint8_t int1_tpulse_duration: 3;
	uint8_t resv_1: 5;
} int_pulse_min_on_intf1_internal_t;

#define INT_PULSE_MIN_OFF_INTF0 0xa264
typedef struct {
	uint8_t int0_tdeassert_disable: 3;
	uint8_t resv_1: 5;
} int_pulse_min_off_intf0_internal_t;

#define INT_PULSE_MIN_OFF_INTF1 0xa265
typedef struct {
	uint8_t int1_tdeassert_disable: 3;
	uint8_t resv_1: 5;
} int_pulse_min_off_intf1_internal_t;

#define ISR_0_7 0xa267
typedef struct {
	uint8_t int_status_accel_drdy_pin_0: 1;
	uint8_t int_status_gyro_drdy_pin_0: 1;
	uint8_t int_status_ext_int_drdy_pin_0: 1;
	uint8_t int_status_ext_odr_drdy_pin_0: 1;
	uint8_t int_status_wom_drdy_pin_0: 1;
	uint8_t int_status_on_demand_pin_0: 1;
	uint8_t resv_1: 2;
} isr_0_7_internal_t;

#define ISR_8_15 0xa268
typedef struct {
	uint8_t int_status_accel_drdy_pin_1: 1;
	uint8_t int_status_gyro_drdy_pin_1: 1;
	uint8_t int_status_ext_int_drdy_pin_1: 1;
	uint8_t int_status_ext_odr_drdy_pin_1: 1;
	uint8_t int_status_wom_drdy_pin_1: 1;
	uint8_t int_status_on_demand_pin_1: 1;
	uint8_t resv_1: 2;
} isr_8_15_internal_t;

#define ISR_16_23 0xa269
typedef struct {
	uint8_t int_status_accel_drdy_pin_2: 1;
	uint8_t int_status_gyro_drdy_pin_2: 1;
	uint8_t int_status_ext_int_drdy_pin_2: 1;
	uint8_t int_status_ext_odr_drdy_pin_2: 1;
	uint8_t int_status_wom_drdy_pin_2: 1;
	uint8_t int_status_on_demand_pin_2: 1;
	uint8_t resv_1: 2;
} isr_16_23_internal_t;

#define STATUS_MASK_PIN_0_7 0xa26a
typedef struct {
	uint8_t int_accel_drdy_pin_0_dis: 1;
	uint8_t int_gyro_drdy_pin_0_dis: 1;
	uint8_t int_ext_int_drdy_pin_0_dis: 1;
	uint8_t int_ext_odr_drdy_pin_0_dis: 1;
	uint8_t int_wom_drdy_pin_0_dis: 1;
	uint8_t int_on_demand_pin_0_dis: 1;
	uint8_t resv_1: 2;
} status_mask_pin_0_7_internal_t;

#define STATUS_MASK_PIN_8_15 0xa26b
typedef struct {
	uint8_t int_accel_drdy_pin_1_dis: 1;
	uint8_t int_gyro_drdy_pin_1_dis: 1;
	uint8_t int_ext_int_drdy_pin_1_dis: 1;
	uint8_t int_ext_odr_drdy_pin_1_dis: 1;
	uint8_t int_wom_drdy_pin_1_dis: 1;
	uint8_t int_on_demand_pin_1_dis: 1;
	uint8_t resv_1: 2;
} status_mask_pin_8_15_internal_t;

#define STATUS_MASK_PIN_16_23 0xa26c
typedef struct {
	uint8_t int_accel_drdy_pin_2_dis: 1;
	uint8_t int_gyro_drdy_pin_2_dis: 1;
	uint8_t int_ext_int_drdy_pin_2_dis: 1;
	uint8_t int_ext_odr_drdy_pin_2_dis: 1;
	uint8_t int_wom_drdy_pin_2_dis: 1;
	uint8_t int_on_demand_pin_2_dis: 1;
	uint8_t resv_1: 2;
} status_mask_pin_16_23_internal_t;

#define INT_I2CM_SOURCE 0xa26d
typedef struct {
	uint8_t int_status_i2cm_ioc_ext_trig_en: 1;
	uint8_t int_status_i2cm_smc_ext_odr_en: 1;
	uint8_t resv_1: 6;
} int_i2cm_source_internal_t;

#define PSEQ_FORCE_CTRL_0 0xa26e
typedef struct {
	uint8_t en_dvdd_sw_comp_d2a: 2;
	uint8_t en_dvdd_sw_d2a: 2;
	uint8_t en_gyro_drv_dcc_d2a: 2;
	uint8_t en_gyro_drv_dig_phs_d2a: 2;
} pseq_force_ctrl_0_internal_t;

#define PSEQ_FORCE_CTRL_1 0xa26f
typedef struct {
	uint8_t en_gyro_pll_calib_d2a: 2;
	uint8_t en_gyro_pll_dcc_d2a: 2;
	uint8_t gyro_dmd_sync_b_d2d: 2;
	uint8_t gyro_drv_agc_on_a2d: 2;
} pseq_force_ctrl_1_internal_t;

#define PSEQ_FORCE_CTRL_2 0xa270
typedef struct {
	uint8_t gx_sc2v_mems_disconnect_d2a: 2;
	uint8_t gy_sc2v_mems_disconnect_d2a: 2;
	uint8_t gz_sc2v_mems_disconnect_d2a: 2;
	uint8_t gyro_pll_ref_clk_rdy_a2d: 2;
} pseq_force_ctrl_2_internal_t;

#define PSEQ_FORCE_CTRL_3 0xa271
typedef struct {
	uint8_t pseq_iso_en: 2;
	uint8_t pseq_sw_clk_en: 2;
	uint8_t rst_gyro_dmd_b_d2d: 2;
	uint8_t en_clk_gyro_2p048m_d2d: 1;
	uint8_t en_clk_gyro_512k_d2a: 1;
} pseq_force_ctrl_3_internal_t;

#define PSEQ_FORCE_CTRL_4 0xa272
typedef struct {
	uint8_t en_clk_gyro_6p144m_d2d: 1;
	uint8_t gyro_cp25v_cprdy_a2d: 1;
	uint8_t gyro_drv_idle_state_a2d: 1;
	uint8_t pd_gx_dmd_b_d2a: 1;
	uint8_t pd_gy_dmd_b_d2a: 1;
	uint8_t pd_gz_dmd_b_d2a: 1;
	uint8_t pd_gx_sc2v_b_d2a: 1;
	uint8_t pd_gy_sc2v_b_d2a: 1;
} pseq_force_ctrl_4_internal_t;

#define PSEQ_FORCE_CTRL_5 0xa273
typedef struct {
	uint8_t pd_gz_sc2v_b_d2a: 1;
	uint8_t pd_gyro_bias_master_b_d2a: 1;
	uint8_t pd_gyro_cp25v_b_d2a: 1;
	uint8_t pd_gyro_dc2v_b_d2a: 1;
	uint8_t pd_gyro_drv_b_d2a: 1;
	uint8_t pd_gyro_pll_b_d2a: 1;
	uint8_t pseq_en_gx_dmd_d2d: 1;
	uint8_t pseq_en_gy_dmd_d2d: 1;
} pseq_force_ctrl_5_internal_t;

#define PSEQ_FORCE_CTRL_6 0xa274
typedef struct {
	uint8_t pseq_en_gz_dmd_d2d: 1;
	uint8_t rst_gx_adc_b_d2a: 1;
	uint8_t rst_gy_adc_b_d2a: 1;
	uint8_t rst_gz_adc_b_d2a: 1;
	uint8_t rst_gyro_dc2v_b_d2a: 1;
	uint8_t rst_gyro_drv_b_d2a: 1;
	uint8_t rst_gyro_pll_b_d2a: 1;
	uint8_t resv_1: 1;
} pseq_force_ctrl_6_internal_t;

#define PSEQ_FORCE_CTRL_7 0xa275
typedef struct {
	uint8_t gyro_clk_6p144m_done_dis_trim: 1;
	uint8_t en_clk_tmp_3m_ovrd: 1;
	uint8_t pd_tmp_b_ovrd: 1;
	uint8_t pd_tmp_bias_ovrd: 1;
	uint8_t cmos_only: 1;
	uint8_t resv_1: 3;
} pseq_force_ctrl_7_internal_t;

#define PSEQ_STATUS 0xa276
typedef struct {
	uint8_t sw_pwr_rdy: 1;
	uint8_t resv_1: 7;
} pseq_status_internal_t;

#define ACCEL_WOM_X_THR 0xa277
typedef struct {
	uint8_t wom_x_th: 8;
} accel_wom_x_thr_internal_t;

#define ACCEL_WOM_Y_THR 0xa278
typedef struct {
	uint8_t wom_y_th: 8;
} accel_wom_y_thr_internal_t;

#define ACCEL_WOM_Z_THR 0xa279
typedef struct {
	uint8_t wom_z_th: 8;
} accel_wom_z_thr_internal_t;

#define CKGA_CONFIG 0xa27a
typedef struct {
	uint8_t dis_gyro_sync_2m_rdy: 1;
	uint8_t force_gyro_sync_2m_rdy: 1;
	uint8_t sel_freq_clk_gyro_pll_ref_29k: 2;
	uint8_t resv_1: 4;
} ckga_config_internal_t;

#define CKGA_RCOSC_READY_TRIM 0xa27b
typedef struct {
	uint8_t rcosc_ready_t_trim: 3;
	uint8_t resv_1: 5;
} ckga_rcosc_ready_trim_internal_t;

#define IOC_PADS_TP_OVRD 0xa27c
typedef struct {
	uint8_t tp0_from_pads_ovrd_sel: 1;
	uint8_t tp1_from_pads_ovrd_sel: 1;
	uint8_t tp2_from_pads_ovrd_sel: 1;
	uint8_t tp3_from_pads_ovrd_sel: 1;
	uint8_t tp0_from_pads_ovrd_val: 1;
	uint8_t tp1_from_pads_ovrd_val: 1;
	uint8_t tp2_from_pads_ovrd_val: 1;
	uint8_t tp3_from_pads_ovrd_val: 1;
} ioc_pads_tp_ovrd_internal_t;

#define IOC_PADS_CONFIG0 0xa27d
typedef struct {
	uint8_t tp0_oe: 1;
	uint8_t tp1_oe: 1;
	uint8_t tp2_oe: 1;
	uint8_t tp3_oe: 1;
	uint8_t drdy_intr_polarity: 1;
	uint8_t fsync_polarity: 1;
	uint8_t pads_chop_freq_sel: 2;
} ioc_pads_config0_internal_t;

#define IOC_PADS_CONFIG1 0xa27e
typedef struct {
	uint8_t resv_1: 1;
	uint8_t aux_sdi_tp1_od: 1;
	uint8_t aux_sclk_tp2_od: 1;
	uint8_t aux_cs_tp3_od: 1;
	uint8_t resv_2: 4;
} ioc_pads_config1_internal_t;

#define IOC_VOHVOL 0xa27f
typedef struct {
	uint8_t vohvol_en: 1;
	uint8_t vohvol_sel: 3;
	uint8_t resv_1: 4;
} ioc_vohvol_internal_t;

#define IOC_VIHVIL 0xa280
typedef struct {
	uint8_t tmux_sel: 8;
} ioc_vihvil_internal_t;

#define BAR_CONFIG 0xa281
typedef struct {
	uint8_t bar_rst: 1;
	uint8_t bar_use_otp: 1;
	uint8_t resv_1: 6;
} bar_config_internal_t;

#define IREG_OTP_CFG 0xa282
typedef struct {
	uint8_t otp_copy_mode: 2;
	uint8_t resv_1: 6;
} ireg_otp_cfg_internal_t;

#define IREG_OTP_STATUS 0xa283
typedef struct {
	uint8_t otp_valid_bank: 2;
	uint8_t resv_1: 6;
} ireg_otp_status_internal_t;

#define OTP_CONFIG0 0xa284
typedef struct {
	uint8_t crcerrpro_disable: 1;
	uint8_t resv_1: 7;
} otp_config0_internal_t;

#define OTP_CALCULATED_CRC_0 0xa285
#define OTP_CALCULATED_CRC_1 0xa286
#define HEAT_CTRL1           0xa287
typedef struct {
	uint8_t heater_feature_disable: 1;
	uint8_t resv_1: 7;
} heat_ctrl1_internal_t;

#define HEAT_CTRL2 0xa288
typedef struct {
	uint8_t pd_heater_ana_b: 1;
	uint8_t resv_1: 7;
} heat_ctrl2_internal_t;

#define SELFTEST 0xa289
typedef struct {
	uint8_t en_ax_st: 1;
	uint8_t en_ay_st: 1;
	uint8_t en_az_st: 1;
	uint8_t en_gx_st: 1;
	uint8_t en_gy_st: 1;
	uint8_t en_gz_st: 1;
	uint8_t resv_1: 2;
} selftest_internal_t;

#define GYRO_SC2V 0xa28a
typedef struct {
	uint8_t gyro_sc2v_cont_mode_trim_d2d: 1;
	uint8_t resv_1: 7;
} gyro_sc2v_internal_t;

#define DTMD_DEBUG_CONFIG 0xa28b
typedef struct {
	uint8_t dtmd_en: 1;
	uint8_t dtp_aux1_debug: 1;
	uint8_t resv_1: 2;
	uint8_t dtmd_oe_mask: 4;
} dtmd_debug_config_internal_t;

#define DTMD_OUT_IN_SEL 0xa28c
typedef struct {
	uint8_t dtmd_out_ip_sel: 5;
	uint8_t dtmd_in_mode_sel: 3;
} dtmd_out_in_sel_internal_t;

#define DTMD_OUT_MODE_SEL 0xa28d
typedef struct {
	uint8_t dtmd_out_mode_sel: 6;
	uint8_t resv_1: 2;
} dtmd_out_mode_sel_internal_t;

#define RESERVED_OTP_BYTE3 0xa28e
typedef struct {
	uint8_t reserved_otp_byte3: 1;
	uint8_t resv_1: 7;
} reserved_otp_byte3_internal_t;

#define RESERVED_OTP_BYTE4 0xa28f
typedef struct {
	uint8_t reserved_otp_byte4: 1;
	uint8_t resv_1: 7;
} reserved_otp_byte4_internal_t;

#define IPREG_MISC 0xa290
typedef struct {
	uint8_t general_dbg_en: 1;
	uint8_t edmp_idle: 1;
	uint8_t general_dbg2_en: 1;
	uint8_t resv_1: 5;
} ipreg_misc_internal_t;

#define DIG_SPARE0 0xa291
typedef struct {
	uint8_t dig_spare0: 8;
} dig_spare0_internal_t;

#define DIG_SPARE1 0xa292
typedef struct {
	uint8_t dig_spare1: 8;
} dig_spare1_internal_t;

#define DIG_SPARE2 0xa293
typedef struct {
	uint8_t dig_spare2: 8;
} dig_spare2_internal_t;

#define DIG_SPARE3 0xa294
typedef struct {
	uint8_t dig_spare3: 8;
} dig_spare3_internal_t;

#define DIG_SPARE4 0xa295
typedef struct {
	uint8_t dig_spare4: 8;
} dig_spare4_internal_t;

#define DIG_SPARE5 0xa296
typedef struct {
	uint8_t dig_spare5: 8;
} dig_spare5_internal_t;

#define SW_RCOSC1_TRIM 0xa2a1
typedef struct {
	uint8_t sw_rcosc1_trim: 8;
} sw_rcosc1_trim_internal_t;

#define SW_PLL1_TRIM 0xa2a2
typedef struct {
	uint8_t sw_pll1_trim: 8;
} sw_pll1_trim_internal_t;

#define SA_KEY1_LAST_BYTE 0xa2a3
typedef struct {
	uint8_t sa_key1_last_byte: 8;
} sa_key1_last_byte_internal_t;

#define AUX1_INTF_CONFIG 0xa2a4
typedef struct {
	uint8_t aux1_i3c_sda_out_dly: 3;
	uint8_t resv_1: 5;
} aux1_intf_config_internal_t;

#define FIFO_SRAM_SLEEP 0xa2a7
typedef struct {
	uint8_t fifo_gsleep_shared_sram: 1;
	uint8_t resv_1: 7;
} fifo_sram_sleep_internal_t;

#define FIFO_SRAM_RET 0xa2a8
typedef struct {
	uint8_t fifo_retention_shared_sram: 1;
	uint8_t resv_1: 7;
} fifo_sram_ret_internal_t;

#define EDMP_APEX_CFG0 0xa2aa
typedef struct {
	uint8_t tap_feat_en: 1;
	uint8_t nomotion_feat_en: 1;
	uint8_t shake_feat_en: 1;
	uint8_t tilt_feat_en: 1;
	uint8_t pedo_feat_en: 1;
	uint8_t ff_feat_en: 1;
	uint8_t r2w_feat_en: 1;
	uint8_t smd_feat_en: 1;
} edmp_apex_cfg0_internal_t;

#define EDMP_APEX_CFG1 0xa2ab
typedef struct {
	uint8_t flat_feat_en: 1;
	uint8_t reserved1_feat_en: 1;
	uint8_t reserved2_feat_en: 1;
	uint8_t reserved3_feat_en: 1;
	uint8_t reserved4_feat_en: 1;
	uint8_t b2s_feat_en: 1;
	uint8_t reserved5_feat_en: 1;
	uint8_t resv_1: 1;
} edmp_apex_cfg1_internal_t;

#define OTP_EXPECTED_CRC_0 0xa2fa
#define OTP_EXPECTED_CRC_1 0xa2fb
#define EDMP_SGI_0         0xa2fc
#define EDMP_SGI_1         0xa2fd
#define EDMP_SGI_2         0xa2fe

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
#define EXT_DATA_X        0x10
typedef struct {
	uint8_t ext_gyro_data_x_ap: 4;
	uint8_t ext_accel_data_x_ap: 4;
} ext_data_x_internal_t;

#define EXT_DATA_Y 0x11
typedef struct {
	uint8_t ext_gyro_data_y_ap: 4;
	uint8_t ext_accel_data_y_ap: 4;
} ext_data_y_internal_t;

#define EXT_DATA_Z 0x12
typedef struct {
	uint8_t ext_gyro_data_z_ap: 4;
	uint8_t ext_accel_data_z_ap: 4;
} ext_data_z_internal_t;

#define PWR_MGMT0 0x14
typedef struct {
	uint8_t accel_mode: 2;
	uint8_t gyro_mode: 2;
	uint8_t accel_vre_sel: 1;
	uint8_t accel_lp_clk_sel: 1;
	uint8_t resv_1: 2;
} pwr_mgmt0_internal_t;

#define PWR_MGMT1 0x15
typedef struct {
	uint8_t accel_ui_status_conf_invalid: 1;
	uint8_t gyro_ui_status_conf_invalid: 1;
	uint8_t accel_ui_status_pm: 2;
	uint8_t gyro_ui_status_pm: 2;
	uint8_t gyro_drv_overdrive_status: 1;
	uint8_t resv_1: 1;
} pwr_mgmt1_internal_t;

#define FIFO_COUNT_0 0x16
#define FIFO_COUNT_1 0x17
#define FIFO_DATA    0x18
typedef struct {
	uint8_t fifo_rdata: 8;
} fifo_data_internal_t;

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
} int1_config0_internal_t;

#define INT1_CONFIG1 0x1b
typedef struct {
	uint8_t int1_status_en_pll_rdy: 1;
	uint8_t int1_status_en_wom_x: 1;
	uint8_t int1_status_en_wom_y: 1;
	uint8_t int1_status_en_wom_z: 1;
	uint8_t int1_status_en_i3c_protocol_err: 1;
	uint8_t int1_status_en_i2cm_done: 1;
	uint8_t int1_status_en_apex_event: 1;
	uint8_t resv_1: 1;
} int1_config1_internal_t;

#define INT1_CONFIG2 0x1c
typedef struct {
	uint8_t int1_polarity: 1;
	uint8_t int1_mode: 1;
	uint8_t int1_drive: 1;
	uint8_t resv_1: 5;
} int1_config2_internal_t;

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
} int1_status0_internal_t;

#define INT1_STATUS1 0x1e
typedef struct {
	uint8_t int1_status_pll_rdy: 1;
	uint8_t int1_status_wom_x: 1;
	uint8_t int1_status_wom_y: 1;
	uint8_t int1_status_wom_z: 1;
	uint8_t int1_status_i3c_protocol_err: 1;
	uint8_t int1_status_i2cm_done: 1;
	uint8_t int1_status_apex_event: 1;
	uint8_t resv_1: 1;
} int1_status1_internal_t;

#define ACCEL_CONFIG0 0x1f
typedef struct {
	uint8_t accel_odr: 4;
	uint8_t ap_accel_fs_sel: 3;
	uint8_t resv_1: 1;
} accel_config0_internal_t;

#define GYRO_CONFIG0 0x20
typedef struct {
	uint8_t gyro_odr: 4;
	uint8_t ap_gyro_fs_sel: 4;
} gyro_config0_internal_t;

#define FIFO_CONFIG0 0x21
typedef struct {
	uint8_t fifo_depth: 6;
	uint8_t fifo_mode: 2;
} fifo_config0_internal_t;

#define FIFO_CONFIG1_0 0x22
#define FIFO_CONFIG1_1 0x23
#define FIFO_CONFIG2   0x24
typedef struct {
	uint8_t fifo_frame_size: 3;
	uint8_t fifo_wr_wm_gt_th: 1;
	uint8_t fifo_int_ovfl: 1;
	uint8_t fifo_addr_space_lock: 1;
	uint8_t fifo_bus_intf_sel: 1;
	uint8_t fifo_flush: 1;
} fifo_config2_internal_t;

#define FIFO_CONFIG3 0x25
typedef struct {
	uint8_t fifo_if_en: 1;
	uint8_t fifo_accel_en: 1;
	uint8_t fifo_gyro_en: 1;
	uint8_t fifo_hires_en: 1;
	uint8_t resv_1: 4;
} fifo_config3_internal_t;

#define FIFO_CONFIG4 0x26
typedef struct {
	uint8_t fifo_tmst_fsync_en: 1;
	uint8_t fifo_comp_en: 1;
	uint8_t fifo_comp_nc_flow_cfg: 3;
	uint8_t resv_1: 3;
} fifo_config4_internal_t;

#define TMST_WOM_CONFIG 0x27
typedef struct {
	uint8_t wom_int_dur: 2;
	uint8_t wom_int_mode: 1;
	uint8_t wom_mode: 1;
	uint8_t wom_en: 1;
	uint8_t tmst_resol: 1;
	uint8_t tmst_delta_en: 1;
	uint8_t resv_1: 1;
} tmst_wom_config_internal_t;

#define FSYNC_CONFIG0 0x28
typedef struct {
	uint8_t ap_fsync_sel: 3;
	uint8_t ap_fsync_flag_clear_sel: 1;
	uint8_t resv_1: 4;
} fsync_config0_internal_t;

#define FSYNC_CONFIG1 0x29
typedef struct {
	uint8_t aux1_fsync_sel: 3;
	uint8_t aux1_fsync_flag_clear_sel: 1;
	uint8_t resv_1: 4;
} fsync_config1_internal_t;

#define OTP_HEATER_RTC_CONFIG 0x2a
typedef struct {
	uint8_t otp_done: 1;
	uint8_t otp_crc_pass: 1;
	uint8_t heater_en: 1;
	uint8_t heater_clk_mode: 2;
	uint8_t rtc_mode: 1;
	uint8_t rtc_align: 1;
	uint8_t resv_1: 1;
} otp_heater_rtc_config_internal_t;

#define DMP_EXT_SEN_ODR_CFG 0x2b
typedef struct {
	uint8_t apex_odr: 3;
	uint8_t resv_1: 5;
} dmp_ext_sen_odr_cfg_internal_t;

#define ODR_DECIMATE_CONFIG 0x2c
typedef struct {
	uint8_t accel_fifo_odr_dec: 4;
	uint8_t gyro_fifo_odr_dec: 4;
} odr_decimate_config_internal_t;

#define EDMP_APEX_EN0 0x2d
typedef struct {
	uint8_t tap_en: 1;
	uint8_t nomotion_en: 1;
	uint8_t shake_en: 1;
	uint8_t tilt_en: 1;
	uint8_t pedo_en: 1;
	uint8_t ff_en: 1;
	uint8_t r2w_en: 1;
	uint8_t smd_en: 1;
} edmp_apex_en0_internal_t;

#define EDMP_APEX_EN1 0x2e
typedef struct {
	uint8_t flat_en: 1;
	uint8_t init_en: 1;
	uint8_t power_save_en: 1;
	uint8_t reserved1: 1;
	uint8_t reserved2: 1;
	uint8_t b2s_en: 1;
	uint8_t edmp_enable: 1;
	uint8_t resv_1: 1;
} edmp_apex_en1_internal_t;

#define APEX_BUFFER_MGMT 0x2f
typedef struct {
	uint8_t step_count_edmp_wptr: 2;
	uint8_t step_count_host_rptr: 2;
	uint8_t ff_duration_edmp_wptr: 2;
	uint8_t ff_duration_host_rptr: 2;
} apex_buffer_mgmt_internal_t;

#define INTF_CONFIG0 0x30
typedef struct {
	uint8_t ap_spi_mode: 1;
	uint8_t ap_spi_34_mode: 1;
	uint8_t ap_spi_fuji_mode: 1;
	uint8_t i23c_timeout_en: 1;
	uint8_t ap_ireg_auto_addr_inc_dis: 1;
	uint8_t virtual_access_aux1_en: 1;
	uint8_t glitch_filter_dis: 1;
	uint8_t resv_1: 1;
} intf_config0_internal_t;

#define INTF_CONFIG1_OVRD 0x31
typedef struct {
	uint8_t ap_spi_mode_ovrd_val: 1;
	uint8_t ap_spi_mode_ovrd: 1;
	uint8_t ap_spi_34_mode_ovrd_val: 1;
	uint8_t ap_spi_34_mode_ovrd: 1;
	uint8_t resv_1: 4;
} intf_config1_ovrd_internal_t;

#define INTF_AUX_CONFIG 0x32
typedef struct {
	uint8_t aux1_spi_mode: 1;
	uint8_t aux1_spi_34_mode: 1;
	uint8_t resv_1: 6;
} intf_aux_config_internal_t;

#define IOC_PAD_SCENARIO 0x33
typedef struct {
	uint8_t aux1_enable: 1;
	uint8_t resv_1: 3;
	uint8_t pads_int2_cfg: 2;
	uint8_t pads_drdy_passthrough: 1;
	uint8_t resv_2: 1;
} ioc_pad_scenario_internal_t;

#define IOC_PAD_SCENARIO_AUX_OVRD 0x34
typedef struct {
	uint8_t aux1_enable_ovrd_val: 1;
	uint8_t aux1_enable_ovrd: 1;
	uint8_t resv_1: 6;
} ioc_pad_scenario_aux_ovrd_internal_t;

#define IOC_PAD_SCENARIO_OVRD 0x35
typedef struct {
	uint8_t pads_int2_cfg_ovrd_val: 2;
	uint8_t pads_int2_cfg_ovrd: 1;
	uint8_t pads_drdy_passthrough_ovrd_val: 1;
	uint8_t pads_drdy_passthrough_ovrd: 1;
	uint8_t resv_1: 3;
} ioc_pad_scenario_ovrd_internal_t;

#define DRIVE_CONFIG0 0x36
typedef struct {
	uint8_t pads_spi_slew: 3;
	uint8_t pads_i2c_slew: 3;
	uint8_t resv_1: 2;
} drive_config0_internal_t;

#define DRIVE_CONFIG1 0x37
typedef struct {
	uint8_t pads_i3c_sdr_slew: 3;
	uint8_t pads_i3c_ddr_slew: 3;
	uint8_t resv_1: 2;
} drive_config1_internal_t;

#define DRIVE_CONFIG2 0x38
typedef struct {
	uint8_t pads_slew: 3;
	uint8_t resv_1: 5;
} drive_config2_internal_t;

#define REG_MISC1 0x39
typedef struct {
	uint8_t osc_id_ovrd: 4;
	uint8_t osc_force_on: 4;
} reg_misc1_internal_t;

#define REG_MISC3 0x3a
typedef struct {
	uint8_t wakeup_mclk: 1;
	uint8_t ref_force_on: 2;
	uint8_t edosc_force_on: 1;
	uint8_t resv_1: 4;
} reg_misc3_internal_t;

#define REG_MISC4 0x3b
typedef struct {
	uint8_t req_ref_stb_ovrd: 5;
	uint8_t resv_1: 3;
} reg_misc4_internal_t;

#define REG_MISC0 0x3c
typedef struct {
	uint8_t req_clk_all: 1;
	uint8_t dft_mode: 4;
	uint8_t resv_1: 3;
} reg_misc0_internal_t;

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
} int_apex_config0_internal_t;

#define INT_APEX_CONFIG1 0x3e
typedef struct {
	uint8_t int_status_mask_pin_r2w_sleep_det: 1;
	uint8_t int_status_mask_pin_smd_det: 1;
	uint8_t int_status_mask_pin_selftest_done: 1;
	uint8_t int_status_mask_pin_selfcalib_done: 1;
	uint8_t int_status_mask_pin_sa_done: 1;
	uint8_t int_status_mask_pin_shake_det: 1;
	uint8_t int_status_mask_pin_b2s_det: 1;
	uint8_t int_status_mask_pin_revb2s_det: 1;
} int_apex_config1_internal_t;

#define INT_APEX_CONFIG2 0x3f
typedef struct {
	uint8_t int_status_mask_pin_nomotion_det: 1;
	uint8_t int_status_mask_pin_motion_det: 1;
	uint8_t int_status_mask_pin_noflat_det: 1;
	uint8_t int_status_mask_pin_flat_det: 1;
	uint8_t resv_1: 4;
} int_apex_config2_internal_t;

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
} int_apex_status0_internal_t;

#define INT_APEX_STATUS1 0x41
typedef struct {
	uint8_t int_status_r2w_sleep_det: 1;
	uint8_t int_status_smd_det: 1;
	uint8_t int_status_selftest_done: 1;
	uint8_t int_status_selfcalib_done: 1;
	uint8_t int_status_sa_done: 1;
	uint8_t int_status_shake_det: 1;
	uint8_t int_status_b2s_det: 1;
	uint8_t int_status_revb2s_det: 1;
} int_apex_status1_internal_t;

#define INT_APEX_STATUS2 0x42
typedef struct {
	uint8_t int_status_nomotion_det: 1;
	uint8_t int_status_motion_det: 1;
	uint8_t int_status_noflat_det: 1;
	uint8_t int_status_flat_det: 1;
	uint8_t resv_1: 4;
} int_apex_status2_internal_t;

#define DIG_SPARE6 0x43
typedef struct {
	uint8_t dig_spare6: 8;
} dig_spare6_internal_t;

#define DIG_SPARE7 0x44
typedef struct {
	uint8_t dig_spare7: 8;
} dig_spare7_internal_t;

#define INTF_CONFIG_OVRD_AUX1 0x46
typedef struct {
	uint8_t aux1_spi_mode_ovrd_val: 1;
	uint8_t aux1_spi_mode_ovrd: 1;
	uint8_t aux1_spi_34_mode_ovrd_val: 1;
	uint8_t aux1_spi_34_mode_ovrd: 1;
	uint8_t aux1_ireg_auto_addr_inc_dis: 1;
	uint8_t resv_1: 3;
} intf_config_ovrd_aux1_internal_t;

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
	uint8_t accel_aux1_status_pm: 1;
	uint8_t gyro_aux1_status_pm: 1;
	uint8_t resv_1: 4;
} pwr_mgmt_aux1_internal_t;

#define FS_SEL_AUX1 0x59
typedef struct {
	uint8_t aux1_accel_fs_sel: 3;
	uint8_t aux1_gyro_fs_sel: 4;
	uint8_t resv_1: 1;
} fs_sel_aux1_internal_t;

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
} int2_config0_internal_t;

#define INT2_CONFIG1 0x5b
typedef struct {
	uint8_t int2_status_en_pll_rdy: 1;
	uint8_t int2_status_en_wom_x: 1;
	uint8_t int2_status_en_wom_y: 1;
	uint8_t int2_status_en_wom_z: 1;
	uint8_t int2_status_en_i3c_protocol_err: 1;
	uint8_t int2_status_en_i2cm_done: 1;
	uint8_t int2_status_en_apex_event: 1;
	uint8_t resv_1: 1;
} int2_config1_internal_t;

#define INT2_CONFIG2 0x5c
typedef struct {
	uint8_t int2_polarity: 1;
	uint8_t int2_mode: 1;
	uint8_t int2_drive: 1;
	uint8_t resv_1: 5;
} int2_config2_internal_t;

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
} int2_status0_internal_t;

#define INT2_STATUS1 0x5e
typedef struct {
	uint8_t int2_status_pll_rdy: 1;
	uint8_t int2_status_wom_x: 1;
	uint8_t int2_status_wom_y: 1;
	uint8_t int2_status_wom_z: 1;
	uint8_t int2_status_i3c_protocol_err: 1;
	uint8_t int2_status_i2cm_done: 1;
	uint8_t int2_status_apex_event: 1;
	uint8_t resv_1: 1;
} int2_status1_internal_t;

#define EDMP_GPR_0    0x6c
#define EDMP_GPR_1    0x6d
#define EDMP_GPR_2    0x6e
#define EDMP_GPR_3    0x6f
#define EDMP_GPR_CTRL 0x70
typedef struct {
	uint8_t edmp_gpr_ctrl: 4;
	uint8_t resv_1: 4;
} edmp_gpr_ctrl_internal_t;

#define WHO_AM_I 0x72
typedef struct {
	uint8_t who_am_i: 8;
} who_am_i_internal_t;

#define REG_HOST_MSG 0x73
typedef struct {
	uint8_t testopenable: 1;
	uint8_t reserved1: 1;
	uint8_t reserved2: 1;
	uint8_t reserved3: 1;
	uint8_t sa_en: 1;
	uint8_t edmp_on_demand_en: 1;
	uint8_t reserved4: 2;
} reg_host_msg_internal_t;

#define RESERVED_OTP_BYTE0 0x74
typedef struct {
	uint8_t reserved_otp_byte0: 1;
	uint8_t resv_1: 7;
} reserved_otp_byte0_internal_t;

#define RESERVED_OTP_BYTE1 0x75
typedef struct {
	uint8_t reserved_otp_byte1: 1;
	uint8_t resv_1: 7;
} reserved_otp_byte1_internal_t;

#define INTF_COMM_ERR 0x78
typedef struct {
	uint8_t i3c_s0_err: 1;
	uint8_t i3c_ccc_parity_err: 1;
	uint8_t i3c_sdr_parity_err: 1;
	uint8_t i3c_ddr_parity_err: 1;
	uint8_t i3c_ddr_framing_err: 1;
	uint8_t i3c_crc_err: 1;
	uint8_t resv_1: 2;
} intf_comm_err_internal_t;

#define IREG_ADDR_31_24 0x7a
typedef struct {
	uint8_t ireg_addr_31_24: 8;
} ireg_addr_31_24_internal_t;

#define IREG_ADDR_23_16 0x7b
typedef struct {
	uint8_t ireg_spare: 8;
} ireg_addr_23_16_internal_t;

#define IREG_ADDR_15_8 0x7c
typedef struct {
	uint8_t ireg_addr_15_8: 8;
} ireg_addr_15_8_internal_t;

#define IREG_ADDR_7_0 0x7d
typedef struct {
	uint8_t ireg_addr_7_0: 8;
} ireg_addr_7_0_internal_t;

#define IREG_DATA 0x7e
typedef struct {
	uint8_t ireg_data: 8;
} ireg_data_internal_t;

#define REG_MISC2 0x7f
typedef struct {
	uint8_t ireg_done: 1;
	uint8_t soft_rst: 1;
	uint8_t resv_1: 6;
} reg_misc2_internal_t;

#ifdef __cplusplus
}
#endif

#endif /*#ifndef _VENICE_A1_CHIPTOP_202212210831_6469331EA6B_INTERNAL_H_*/
