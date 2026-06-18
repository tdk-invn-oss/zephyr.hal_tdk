/*
 * Copyright (c) 2017 TDK Invensense
 *
 * SPDX-License-Identifier: BSD 3-Clause
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
	uint8_t pads_ap_sclk_pud_trim_d2a: 1;
	uint8_t pads_ap_sclk_pe_trim_d2a: 1;
	uint8_t resv_2: 1;
	uint8_t pads_ap_cs_pud_trim_d2a: 1;
	uint8_t pads_ap_cs_pe_trim_d2a: 1;
	uint8_t resv_1: 3;
} ipreg_bar_reg_56_t;

#define IPREG_BAR_REG_57 0xa039
typedef struct {
	uint8_t pads_aux_cs_tp3_tp_pe_trim_d2a: 1;
	uint8_t resv_3: 1;
	uint8_t pads_ap_sdo_pud_trim_d2a: 1;
	uint8_t pads_ap_sdo_pe_trim_d2a: 1;
	uint8_t resv_2: 1;
	uint8_t pads_ap_sdi_pud_trim_d2a: 1;
	uint8_t pads_ap_sdi_pe_trim_d2a: 1;
	uint8_t resv_1: 1;
} ipreg_bar_reg_57_t;

#define IPREG_BAR_REG_58 0xa03a
typedef struct {
	uint8_t resv_3: 1;
	uint8_t pads_aux_sdi_tp1_tp_pud_trim_d2a: 1;
	uint8_t pads_aux_sdi_tp1_tp_pe_trim_d2a: 1;
	uint8_t resv_2: 1;
	uint8_t pads_aux_sclk_tp2_tp_pud_trim_d2a: 1;
	uint8_t pads_aux_sclk_tp2_tp_pe_trim_d2a: 1;
	uint8_t resv_1: 1;
	uint8_t pads_aux_cs_tp3_tp_pud_trim_d2a: 1;
} ipreg_bar_reg_58_t;

#define IPREG_BAR_REG_59 0xa03b
typedef struct {
	uint8_t pads_int2_pud_trim_d2a: 1;
	uint8_t pads_int2_pe_trim_d2a: 1;
	uint8_t resv_2: 1;
	uint8_t pads_int1_tp0_tp_pud_trim_d2a: 1;
	uint8_t pads_int1_tp0_tp_pe_trim_d2a: 1;
	uint8_t resv_1: 1;
	uint8_t pads_aux_sdo_pud_trim_d2a: 1;
	uint8_t pads_aux_sdo_pe_trim_d2a: 1;
} ipreg_bar_reg_59_t;

#define IPREG_BAR_REG_69 0xa045
typedef struct {
	uint8_t resv_1: 6;
	uint8_t pads_od_pud_strength_trim_d2a: 2;
} ipreg_bar_reg_69_t;

#define IPREG_BAR_REG_81 0xa051
typedef struct {
	uint8_t resv_2: 5;
	uint8_t pd_minibg_b_d2a: 1;
	uint8_t pd_mb_vbgr_test_b_d2a: 1;
	uint8_t resv_1: 1;
} ipreg_bar_reg_81_t;

#define IPREG_BAR_REG_83 0xa053
typedef struct {
	uint8_t resv_1: 7;
	uint8_t io_well_bias_comp_sel_d2a: 1;
} ipreg_bar_reg_83_t;

/* IPREG_ANA */
#define IPREG_ANA_REG_5 0xa305
typedef struct {
	uint8_t resv_1: 2;
	uint8_t gyro_drv_zst_mode_d2a: 2;
	uint8_t gyro_drv_yst_mode_d2a: 2;
	uint8_t gyro_drv_xst_mode_d2a: 2;
} ipreg_ana_reg_5_t;

#define IPREG_ANA_REG_6 0xa306
typedef struct {
	uint8_t resv_1: 5;
	uint8_t gyro_drv_zst_pol_d2a: 1;
	uint8_t gyro_drv_yst_pol_d2a: 1;
	uint8_t gyro_drv_xst_pol_d2a: 1;
} ipreg_ana_reg_6_t;

#define IPREG_ANA_REG_8 0xa308
typedef struct {
	uint8_t resv_1: 6;
	uint8_t pd_accel_st_b_d2a: 1;
	uint8_t pd_accel_cp45_st_b_d2a: 1;
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
} imem_sram_apex_reg_98_t;

#define IMEM_SRAM_APEX_REG_99 0x63
typedef struct {
	uint8_t tilt_reset_en: 8;
} imem_sram_apex_reg_99_t;

#define IMEM_SRAM_APEX_REG_121 0x79
typedef struct {
	uint8_t ped_step_cadence: 8;
} imem_sram_apex_reg_121_t;

#define IMEM_SRAM_APEX_REG_122 0x7a
typedef struct {
	uint8_t ped_activity_class: 8;
} imem_sram_apex_reg_122_t;

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
} imem_sram_apex_reg_142_t;

#define IMEM_SRAM_APEX_REG_143 0x8f
typedef struct {
	uint8_t tap_axis: 8;
} imem_sram_apex_reg_143_t;

#define IMEM_SRAM_APEX_REG_144 0x90
typedef struct {
	uint8_t tap_dir: 8;
} imem_sram_apex_reg_144_t;

#define IMEM_SRAM_APEX_REG_145 0x91
typedef struct {
	uint8_t double_tap_timing: 8;
} imem_sram_apex_reg_145_t;

#define IMEM_SRAM_APEX_REG_146 0x92
typedef struct {
	uint8_t triple_tap_timing: 8;
} imem_sram_apex_reg_146_t;

#define IMEM_SRAM_APEX_REG_147 0x93
typedef struct {
	uint8_t shake_outaxis: 8;
} imem_sram_apex_reg_147_t;

#define IMEM_SRAM_APEX_REG_148 0x94
typedef struct {
	uint8_t nomotion_outlock: 8;
} imem_sram_apex_reg_148_t;

#define IMEM_SRAM_APEX_REG_149 0x95
typedef struct {
	uint8_t nomotion_majority_axis: 8;
} imem_sram_apex_reg_149_t;

#define IMEM_SRAM_APEX_REG_150 0x96
typedef struct {
	uint8_t nomotion_sign_angle: 8;
} imem_sram_apex_reg_150_t;

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
} imem_sram_apex_reg_230_t;

#define IMEM_SRAM_APEX_REG_231 0xe7
typedef struct {
	uint8_t tap_min_jerk: 8;
} imem_sram_apex_reg_231_t;

#define IMEM_SRAM_APEX_REG_232 0xe8
typedef struct {
	uint8_t tap_smudge_reject_thr: 8;
} imem_sram_apex_reg_232_t;

#define IMEM_SRAM_APEX_REG_233 0xe9
typedef struct {
	uint8_t tap_max_peak_tol: 8;
} imem_sram_apex_reg_233_t;

#define IMEM_SRAM_APEX_REG_234 0xea
typedef struct {
	uint8_t tap_tavg: 8;
} imem_sram_apex_reg_234_t;

#define IMEM_SRAM_APEX_REG_236 0xec
typedef struct {
	uint8_t tap_max_tap: 8;
} imem_sram_apex_reg_236_t;

#define IMEM_SRAM_APEX_REG_237 0xed
typedef struct {
	uint8_t tap_min_tap: 8;
} imem_sram_apex_reg_237_t;

#define IMEM_SRAM_APEX_REG_238 0xee
#define IMEM_SRAM_APEX_REG_239 0xef
#define IMEM_SRAM_APEX_REG_240 0xf0
#define IMEM_SRAM_APEX_REG_241 0xf1
#define IMEM_SRAM_APEX_REG_242 0xf2
typedef struct {
	uint8_t tap_axis_select_mask: 8;
} imem_sram_apex_reg_242_t;

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
} imem_sram_apex_reg_912_t;

#define IMEM_SRAM_APEX_REG_938 0x3aa
typedef struct {
	uint8_t smd_sensitivity: 8;
} imem_sram_apex_reg_938_t;

#define IMEM_SRAM_APEX_REG_1068 0x42c
typedef struct {
	uint8_t b2s_mounting_matrix: 8;
} imem_sram_apex_reg_1068_t;

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
	uint8_t resv_1: 2;
	uint8_t indirect_access_en: 1;
	uint8_t mstr_spi_freq: 3;
	uint8_t sent_to_slave: 1;
	uint8_t mstr_spi_rst: 1;
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
	uint8_t resv_1: 6;
	uint8_t mstr_spi_done: 1;
	uint8_t mstr_spi_idle: 1;
} mstr_spi_status_t;

#define SENSOR_DATA_CTRL 0xa605
typedef struct {
	uint8_t resv_1: 4;
	uint8_t gyro_sensor_data_en: 1;
	uint8_t acc_sensor_data_en: 1;
	uint8_t prologue_en: 1;
	uint8_t start_sensor: 1;
} sensor_data_ctrl_t;

#define FPGA_ID 0xa606
typedef struct {
	uint8_t fpga_id: 8;
} fpga_id_t;

#define FPGA_CONFIG 0xa607
typedef struct {
	uint8_t resv_1: 7;
	uint8_t fpga_root_clk_sel: 1;
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
	uint8_t accel_x_pha: 7;
	uint8_t accel_x_dith_en: 1;
} accel_x_pha_t;

#define ACCEL_X_Y_AEX 0xa60e
typedef struct {
	uint8_t accel_x_aex: 4;
	uint8_t accel_y_aex: 4;
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
	uint8_t accel_y_pha: 7;
	uint8_t accel_y_dith_en: 1;
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
	uint8_t accel_z_pha: 7;
	uint8_t accel_z_dith_en: 1;
} accel_z_pha_t;

#define ACCEL_Z_GYRO_X_AEX 0xa617
typedef struct {
	uint8_t accel_z_aex: 4;
	uint8_t gyro_x_aex: 4;
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
	uint8_t gyro_x_pha: 7;
	uint8_t gyro_x_dith_en: 1;
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
	uint8_t gyro_y_pha: 7;
	uint8_t gyro_y_dith_en: 1;
} gyro_y_pha_t;

#define GYRO_Y_GYRO_Z_AEX 0xa620
typedef struct {
	uint8_t gyro_y_aex: 4;
	uint8_t gyro_z_aex: 4;
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
	uint8_t gyro_z_pha: 7;
	uint8_t gyro_z_dith_en: 1;
} gyro_z_pha_t;

#define ACCEL_GYRO_CLKDIV 0xa625
typedef struct {
	uint8_t accel_clk_div: 4;
	uint8_t gyro_clk_div: 4;
} accel_gyro_clkdiv_t;

#define FPGA_CTRL 0xa627
typedef struct {
	uint8_t resv_1: 6;
	uint8_t ram_ovf: 1;
	uint8_t bram_wr_dis: 1;
} fpga_ctrl_t;

#define OTP_ERROR_STATUS 0xa628
typedef struct {
	uint8_t resv_1: 4;
	uint8_t cfg_mismatch_error: 1;
	uint8_t prog_0_bit_error: 1;
	uint8_t prog_more_than_1_bit_error: 1;
	uint8_t diff_read_mismatch_error: 1;
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
	uint8_t gyro_x_tmid_off: 4;
	uint8_t gyro_x_tmid_gain: 4;
} ipreg_sys1_reg_146_t;

#define IPREG_SYS1_REG_148 0xa494
typedef struct {
	uint8_t gyro_y_tmid_off: 4;
	uint8_t gyro_y_tmid_gain: 4;
} ipreg_sys1_reg_148_t;

#define IPREG_SYS1_REG_150 0xa496
typedef struct {
	uint8_t gyro_z_tmid_off: 4;
	uint8_t gyro_z_tmid_gain: 4;
} ipreg_sys1_reg_150_t;
#define IPREG_SYS1_REG_154 0xa49a
typedef struct {
	uint8_t resv_2: 2;
	uint8_t gyro_ois_m6_byp: 1;
	uint8_t gyro_afsr_shared: 1;
	uint8_t gyro_src_ctrl: 2;
	uint8_t resv_1: 1;
	uint8_t gyro_afsr_enable: 1;
} ipreg_sys1_reg_154_t;

#define IPREG_SYS1_REG_155 0xa49b
typedef struct {
	uint8_t resv_1: 6;
	uint8_t gyro_lpf_bypass: 1;
	uint8_t gyro_ois_hpf_byp: 1;
} ipreg_sys1_reg_155_t;

#define IPREG_SYS1_REG_157 0xa49d
typedef struct {
	uint8_t gyro_notch_bypass: 1;
	uint8_t gyro_ois_hpfbw_sel: 3;
	uint8_t gyro_lp_avg_sel: 4;
} ipreg_sys1_reg_157_t;

#define IPREG_SYS1_REG_158 0xa49e
typedef struct {
	uint8_t gyro_ui_3rd_ord_sel: 1;
	uint8_t gyro_ui_lpfbw_sel: 3;
	uint8_t gyro_ois_3rd_ord_sel: 1;
	uint8_t gyro_ois_lpfbw_sel: 3;
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
	uint8_t accel_x_tmid_off: 4;
	uint8_t accel_x_tmid_gain: 4;
} ipreg_sys2_reg_105_t;

#define IPREG_SYS2_REG_106 0xa56a
typedef struct {
	uint8_t accel_y_tmid_off: 4;
	uint8_t accel_y_tmid_gain: 4;
} ipreg_sys2_reg_106_t;

#define IPREG_SYS2_REG_107 0xa56b
typedef struct {
	uint8_t accel_z_tmid_off: 4;
	uint8_t accel_z_tmid_gain: 4;
} ipreg_sys2_reg_107_t;

#define IPREG_SYS2_REG_109 0xa56d
typedef struct {
	uint8_t resv_1: 6;
	uint8_t accel_src_ctrl: 2;
} ipreg_sys2_reg_109_t;

#define IPREG_SYS2_REG_110 0xa56e
typedef struct {
	uint8_t resv_1: 1;
	uint8_t accel_ois_hpfbw_sel: 3;
	uint8_t accel_lp_avg_sel: 4;
} ipreg_sys2_reg_110_t;

#define IPREG_SYS2_REG_111 0xa56f
typedef struct {
	uint8_t resv_1: 4;
	uint8_t accel_ois_3rd_ord_sel: 1;
	uint8_t accel_ois_lpfbw_sel: 3;
} ipreg_sys2_reg_111_t;

#define IPREG_SYS2_REG_112 0xa570
typedef struct {
	uint8_t resv_1: 1;
	uint8_t accel_ois_m6_byp: 1;
	uint8_t accel_ois_hpf_byp: 1;
	uint8_t accel_lpf_bypass: 1;
	uint8_t accel_ui_3rd_ord_sel: 1;
	uint8_t accel_ui_lpfbw_sel: 3;
} ipreg_sys2_reg_112_t;

#define IPREG_SYS2_REG_117 0xa575
typedef struct {
	uint8_t resv_1: 1;
	uint8_t tmp_dec_cfg: 3;
	uint8_t tmp_lpf_cfg: 3;
	uint8_t tmp_inj_en_gos: 1;
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
	uint8_t resv_1: 5;
	uint8_t tmst_force_aux_fine_en: 1;
	uint8_t tmst_fsync_en: 1;
	uint8_t tmst_en: 1;
} smc_control_0_t;

#define SREG_CTRL 0xa260
typedef struct {
	uint8_t resv_1: 4;
	uint8_t sreg_sifs_20bits_en: 1;
	uint8_t sreg_debug_data_inject_en: 1;
	uint8_t sreg_data_endian_sel: 1;
	uint8_t sreg_invalid_ind_dis: 1;
} sreg_ctrl_t;

#define INT_PULSE_MIN_ON_INTF0 0xa262
typedef struct {
	uint8_t resv_1: 5;
	uint8_t int0_tpulse_duration: 3;
} int_pulse_min_on_intf0_t;

#define INT_PULSE_MIN_ON_INTF1 0xa263
typedef struct {
	uint8_t resv_1: 5;
	uint8_t int1_tpulse_duration: 3;
} int_pulse_min_on_intf1_t;

#define INT_PULSE_MIN_OFF_INTF0 0xa264
typedef struct {
	uint8_t resv_1: 5;
	uint8_t int0_tdeassert_disable: 3;
} int_pulse_min_off_intf0_t;

#define INT_PULSE_MIN_OFF_INTF1 0xa265
typedef struct {
	uint8_t resv_1: 5;
	uint8_t int1_tdeassert_disable: 3;
} int_pulse_min_off_intf1_t;

#define STATUS_MASK_PIN_0_7 0xa26a
typedef struct {
	uint8_t resv_2: 2;
	uint8_t int_on_demand_pin_0_dis: 1;
	uint8_t int_wom_drdy_pin_0_dis: 1;
	uint8_t resv_1: 2;
	uint8_t int_gyro_drdy_pin_0_dis: 1;
	uint8_t int_accel_drdy_pin_0_dis: 1;
} status_mask_pin_0_7_t;

#define STATUS_MASK_PIN_8_15 0xa26b
typedef struct {
	uint8_t resv_2: 2;
	uint8_t int_on_demand_pin_1_dis: 1;
	uint8_t int_wom_drdy_pin_1_dis: 1;
	uint8_t resv_1: 2;
	uint8_t int_gyro_drdy_pin_1_dis: 1;
	uint8_t int_accel_drdy_pin_1_dis: 1;
} status_mask_pin_8_15_t;

#define STATUS_MASK_PIN_16_23 0xa26c
typedef struct {
	uint8_t resv_2: 2;
	uint8_t int_on_demand_pin_2_dis: 1;
	uint8_t int_wom_drdy_pin_2_dis: 1;
	uint8_t resv_1: 2;
	uint8_t int_gyro_drdy_pin_2_dis: 1;
	uint8_t int_accel_drdy_pin_2_dis: 1;
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
	uint8_t resv_2: 2;
	uint8_t fsync_polarity: 1;
	uint8_t resv_1: 5;
} ioc_pads_config0_t;

#define IREG_OTP_CFG 0xa282
typedef struct {
	uint8_t resv_1: 6;
	uint8_t otp_copy_mode: 2;
} ireg_otp_cfg_t;

#define SELFTEST 0xa289
typedef struct {
	uint8_t resv_1: 2;
	uint8_t en_gz_st: 1;
	uint8_t en_gy_st: 1;
	uint8_t en_gx_st: 1;
	uint8_t en_az_st: 1;
	uint8_t en_ay_st: 1;
	uint8_t en_ax_st: 1;
} selftest_t;

#define IPREG_MISC 0xa290
typedef struct {
	uint8_t resv_2: 6;
	uint8_t edmp_idle: 1;
	uint8_t resv_1: 1;
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
	uint8_t resv_1: 7;
	uint8_t fifo_gsleep_shared_sram: 1;
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
#define EXT_DATA_X        0x10
typedef struct {
	uint8_t ext_accel_data_x_ap: 4;
	uint8_t ext_gyro_data_x_ap: 4;
} ext_data_x_t;

#define EXT_DATA_Y 0x11
typedef struct {
	uint8_t ext_accel_data_y_ap: 4;
	uint8_t ext_gyro_data_y_ap: 4;
} ext_data_y_t;

#define EXT_DATA_Z 0x12
typedef struct {
	uint8_t ext_accel_data_z_ap: 4;
	uint8_t ext_gyro_data_z_ap: 4;
} ext_data_z_t;

#define PWR_MGMT0 0x14
typedef struct {
	uint8_t resv_2: 2;
	uint8_t accel_lp_clk_sel: 1;
	uint8_t resv_1: 1;
	uint8_t gyro_mode: 2;
	uint8_t accel_mode: 2;
} pwr_mgmt0_t;

#define FIFO_COUNT_0 0x16
#define FIFO_COUNT_1 0x17
#define FIFO_DATA    0x18
typedef struct {
	uint8_t fifo_rdata: 8;
} fifo_data_t;

#define INT1_CONFIG0 0x1a
typedef struct {
	uint8_t int1_status_en_reset_done: 1;
	uint8_t int1_status_en_aux1_agc_rdy: 1;
	uint8_t int1_status_en_ap_agc_rdy: 1;
	uint8_t int1_status_en_ap_fsync: 1;
	uint8_t int1_status_en_aux1_drdy: 1;
	uint8_t int1_status_en_drdy: 1;
	uint8_t int1_status_en_fifo_ths: 1;
	uint8_t int1_status_en_fifo_full: 1;
} int1_config0_t;

#define INT1_CONFIG1 0x1b
typedef struct {
	uint8_t resv_2: 1;
	uint8_t int1_status_en_apex_event: 1;
	uint8_t resv_1: 1;
	uint8_t int1_status_en_i3c_protocol_err: 1;
	uint8_t int1_status_en_wom_z: 1;
	uint8_t int1_status_en_wom_y: 1;
	uint8_t int1_status_en_wom_x: 1;
	uint8_t int1_status_en_pll_rdy: 1;
} int1_config1_t;

#define INT1_CONFIG2 0x1c
typedef struct {
	uint8_t resv_1: 5;
	uint8_t int1_drive: 1;
	uint8_t int1_mode: 1;
	uint8_t int1_polarity: 1;
} int1_config2_t;

#define INT1_STATUS0 0x1d
typedef struct {
	uint8_t int1_status_reset_done: 1;
	uint8_t int1_status_aux1_agc_rdy: 1;
	uint8_t int1_status_ap_agc_rdy: 1;
	uint8_t int1_status_ap_fsync: 1;
	uint8_t int1_status_aux1_drdy: 1;
	uint8_t int1_status_drdy: 1;
	uint8_t int1_status_fifo_ths: 1;
	uint8_t int1_status_fifo_full: 1;
} int1_status0_t;

#define INT1_STATUS1 0x1e
typedef struct {
	uint8_t resv_2: 1;
	uint8_t int1_status_apex_event: 1;
	uint8_t resv_1: 1;
	uint8_t int1_status_i3c_protocol_err: 1;
	uint8_t int1_status_wom_z: 1;
	uint8_t int1_status_wom_y: 1;
	uint8_t int1_status_wom_x: 1;
	uint8_t int1_status_pll_rdy: 1;
} int1_status1_t;

#define ACCEL_CONFIG0 0x1f
typedef struct {
	uint8_t resv_1: 1;
	uint8_t ap_accel_fs_sel: 3;
	uint8_t accel_odr: 4;
} accel_config0_t;

#define GYRO_CONFIG0 0x20
typedef struct {
	uint8_t ap_gyro_fs_sel: 4;
	uint8_t gyro_odr: 4;
} gyro_config0_t;

#define FIFO_CONFIG0 0x21
typedef struct {
	uint8_t fifo_mode: 2;
	uint8_t fifo_depth: 6;
} fifo_config0_t;

#define FIFO_CONFIG1_0 0x22
#define FIFO_CONFIG1_1 0x23
#define FIFO_CONFIG2   0x24
typedef struct {
	uint8_t fifo_flush: 1;
	uint8_t resv_2: 2;
	uint8_t fifo_int_ovfl: 1;
	uint8_t fifo_wr_wm_gt_th: 1;
	uint8_t resv_1: 3;
} fifo_config2_t;

#define FIFO_CONFIG3 0x25
typedef struct {
	uint8_t resv_1: 4;
	uint8_t fifo_hires_en: 1;
	uint8_t fifo_gyro_en: 1;
	uint8_t fifo_accel_en: 1;
	uint8_t fifo_if_en: 1;
} fifo_config3_t;

#define FIFO_CONFIG4 0x26
typedef struct {
	uint8_t resv_1: 3;
	uint8_t fifo_comp_nc_flow_cfg: 3;
	uint8_t fifo_comp_en: 1;
	uint8_t fifo_tmst_fsync_en: 1;
} fifo_config4_t;

#define TMST_WOM_CONFIG 0x27
typedef struct {
	uint8_t resv_1: 1;
	uint8_t tmst_delta_en: 1;
	uint8_t tmst_resol: 1;
	uint8_t wom_en: 1;
	uint8_t wom_mode: 1;
	uint8_t wom_int_mode: 1;
	uint8_t wom_int_dur: 2;
} tmst_wom_config_t;

#define FSYNC_CONFIG0 0x28
typedef struct {
	uint8_t resv_1: 4;
	uint8_t ap_fsync_flag_clear_sel: 1;
	uint8_t ap_fsync_sel: 3;
} fsync_config0_t;

#define FSYNC_CONFIG1 0x29
typedef struct {
	uint8_t resv_1: 4;
	uint8_t aux1_fsync_flag_clear_sel: 1;
	uint8_t aux1_fsync_sel: 3;
} fsync_config1_t;

#define OTP_HEATER_RTC_CONFIG 0x2a
typedef struct {
	uint8_t resv_2: 1;
	uint8_t rtc_align: 1;
	uint8_t rtc_mode: 1;
	uint8_t resv_1: 5;
} otp_heater_rtc_config_t;

#define DMP_EXT_SEN_ODR_CFG 0x2b
typedef struct {
	uint8_t resv_1: 5;
	uint8_t apex_odr: 3;
} dmp_ext_sen_odr_cfg_t;

#define ODR_DECIMATE_CONFIG 0x2c
typedef struct {
	uint8_t gyro_fifo_odr_dec: 4;
	uint8_t accel_fifo_odr_dec: 4;
} odr_decimate_config_t;

#define EDMP_APEX_EN0 0x2d
typedef struct {
	uint8_t smd_en: 1;
	uint8_t r2w_en: 1;
	uint8_t ff_en: 1;
	uint8_t pedo_en: 1;
	uint8_t tilt_en: 1;
	uint8_t shake_en: 1;
	uint8_t nomotion_en: 1;
	uint8_t tap_en: 1;
} edmp_apex_en0_t;

#define EDMP_APEX_EN1 0x2e
typedef struct {
	uint8_t resv_1: 1;
	uint8_t edmp_enable: 1;
	uint8_t b2s_en: 1;
	uint8_t reserved2: 1;
	uint8_t reserved1: 1;
	uint8_t power_save_en: 1;
	uint8_t init_en: 1;
	uint8_t flat_en: 1;
} edmp_apex_en1_t;

#define APEX_BUFFER_MGMT 0x2f
typedef struct {
	uint8_t ff_duration_host_rptr: 2;
	uint8_t ff_duration_edmp_wptr: 2;
	uint8_t step_count_host_rptr: 2;
	uint8_t step_count_edmp_wptr: 2;
} apex_buffer_mgmt_t;

#define INTF_CONFIG0 0x30
typedef struct {
	uint8_t resv_1: 6;
	uint8_t ap_spi_34_mode: 1;
	uint8_t ap_spi_mode: 1;
} intf_config0_t;

#define INTF_CONFIG1_OVRD 0x31
typedef struct {
	uint8_t resv_1: 4;
	uint8_t ap_spi_34_mode_ovrd: 1;
	uint8_t ap_spi_34_mode_ovrd_val: 1;
	uint8_t ap_spi_mode_ovrd: 1;
	uint8_t ap_spi_mode_ovrd_val: 1;
} intf_config1_ovrd_t;

#define INTF_AUX_CONFIG 0x32
typedef struct {
	uint8_t resv_1: 6;
	uint8_t aux1_spi_34_mode: 1;
	uint8_t aux1_spi_mode: 1;
} intf_aux_config_t;

#define IOC_PAD_SCENARIO_OVRD 0x35
typedef struct {
	uint8_t resv_1: 5;
	uint8_t pads_int2_cfg_ovrd: 1;
	uint8_t pads_int2_cfg_ovrd_val: 2;
} ioc_pad_scenario_ovrd_t;

#define DRIVE_CONFIG0 0x36
typedef struct {
	uint8_t resv_1: 2;
	uint8_t pads_i2c_slew: 3;
	uint8_t pads_spi_slew: 3;
} drive_config0_t;

#define DRIVE_CONFIG1 0x37
typedef struct {
	uint8_t resv_1: 2;
	uint8_t pads_i3c_ddr_slew: 3;
	uint8_t pads_i3c_sdr_slew: 3;
} drive_config1_t;

#define DRIVE_CONFIG2 0x38
typedef struct {
	uint8_t resv_1: 5;
	uint8_t pads_slew: 3;
} drive_config2_t;

#define INT_APEX_CONFIG0 0x3d
typedef struct {
	uint8_t int_status_mask_pin_r2w_wake_det: 1;
	uint8_t int_status_mask_pin_ff_det: 1;
	uint8_t int_status_mask_pin_step_det: 1;
	uint8_t int_status_mask_pin_step_cnt_ovfl: 1;
	uint8_t int_status_mask_pin_tilt_det: 1;
	uint8_t int_status_mask_pin_low_g_det: 1;
	uint8_t int_status_mask_pin_high_g_det: 1;
	uint8_t int_status_mask_pin_tap_detect: 1;
} int_apex_config0_t;

#define INT_APEX_CONFIG1 0x3e
typedef struct {
	uint8_t int_status_mask_pin_revb2s_det: 1;
	uint8_t int_status_mask_pin_b2s_det: 1;
	uint8_t int_status_mask_pin_shake_det: 1;
	uint8_t int_status_mask_pin_sa_done: 1;
	uint8_t reserved: 1;
	uint8_t int_status_mask_pin_selftest_done: 1;
	uint8_t int_status_mask_pin_smd_det: 1;
	uint8_t int_status_mask_pin_r2w_sleep_det: 1;
} int_apex_config1_t;

#define INT_APEX_CONFIG2 0x3f
typedef struct {
	uint8_t resv_1: 4;
	uint8_t int_status_mask_pin_flat_det: 1;
	uint8_t int_status_mask_pin_noflat_det: 1;
	uint8_t int_status_mask_pin_motion_det: 1;
	uint8_t int_status_mask_pin_nomotion_det: 1;
} int_apex_config2_t;

#define INT_APEX_STATUS0 0x40
typedef struct {
	uint8_t int_status_r2w_wake_det: 1;
	uint8_t int_status_ff_det: 1;
	uint8_t int_status_step_det: 1;
	uint8_t int_status_step_cnt_ovfl: 1;
	uint8_t int_status_tilt_det: 1;
	uint8_t int_status_low_g_det: 1;
	uint8_t int_status_high_g_det: 1;
	uint8_t int_status_tap_detect: 1;
} int_apex_status0_t;

#define INT_APEX_STATUS1 0x41
typedef struct {
	uint8_t int_status_revb2s_det: 1;
	uint8_t int_status_b2s_det: 1;
	uint8_t int_status_shake_det: 1;
	uint8_t int_status_sa_done: 1;
	uint8_t reserved: 1;
	uint8_t int_status_selftest_done: 1;
	uint8_t int_status_smd_det: 1;
	uint8_t int_status_r2w_sleep_det: 1;
} int_apex_status1_t;

#define INT_APEX_STATUS2 0x42
typedef struct {
	uint8_t resv_1: 4;
	uint8_t int_status_flat_det: 1;
	uint8_t int_status_noflat_det: 1;
	uint8_t int_status_motion_det: 1;
	uint8_t int_status_nomotion_det: 1;
} int_apex_status2_t;

#define INTF_CONFIG_OVRD_AUX1 0x46
typedef struct {
	uint8_t resv_1: 4;
	uint8_t aux1_spi_34_mode_ovrd: 1;
	uint8_t aux1_spi_34_mode_ovrd_val: 1;
	uint8_t aux1_spi_mode_ovrd: 1;
	uint8_t aux1_spi_mode_ovrd_val: 1;
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
	uint8_t resv_1: 6;
	uint8_t gyro_aux1_en: 1;
	uint8_t accel_aux1_en: 1;
} pwr_mgmt_aux1_t;

#define FS_SEL_AUX1 0x59
typedef struct {
	uint8_t resv_1: 1;
	uint8_t aux1_gyro_fs_sel: 4;
	uint8_t aux1_accel_fs_sel: 3;
} fs_sel_aux1_t;

#define INT2_CONFIG0 0x5a
typedef struct {
	uint8_t int2_status_en_reset_done: 1;
	uint8_t int2_status_en_aux1_agc_rdy: 1;
	uint8_t int2_status_en_ap_agc_rdy: 1;
	uint8_t int2_status_en_ap_fsync: 1;
	uint8_t int2_status_en_aux1_drdy: 1;
	uint8_t int2_status_en_drdy: 1;
	uint8_t int2_status_en_fifo_ths: 1;
	uint8_t int2_status_en_fifo_full: 1;
} int2_config0_t;

#define INT2_CONFIG1 0x5b
typedef struct {
	uint8_t resv_2: 1;
	uint8_t int2_status_en_apex_event: 1;
	uint8_t resv_1: 1;
	uint8_t int2_status_en_i3c_protocol_err: 1;
	uint8_t int2_status_en_wom_z: 1;
	uint8_t int2_status_en_wom_y: 1;
	uint8_t int2_status_en_wom_x: 1;
	uint8_t int2_status_en_pll_rdy: 1;
} int2_config1_t;

#define INT2_CONFIG2 0x5c
typedef struct {
	uint8_t resv_1: 5;
	uint8_t int2_drive: 1;
	uint8_t int2_mode: 1;
	uint8_t int2_polarity: 1;
} int2_config2_t;

#define INT2_STATUS0 0x5d
typedef struct {
	uint8_t int2_status_reset_done: 1;
	uint8_t int2_status_aux1_agc_rdy: 1;
	uint8_t int2_status_ap_agc_rdy: 1;
	uint8_t int2_status_ap_fsync: 1;
	uint8_t int2_status_aux1_drdy: 1;
	uint8_t int2_status_drdy: 1;
	uint8_t int2_status_fifo_ths: 1;
	uint8_t int2_status_fifo_full: 1;
} int2_status0_t;

#define INT2_STATUS1 0x5e
typedef struct {
	uint8_t resv_2: 1;
	uint8_t int2_status_apex_event: 1;
	uint8_t resv_1: 1;
	uint8_t int2_status_i3c_protocol_err: 1;
	uint8_t int2_status_wom_z: 1;
	uint8_t int2_status_wom_y: 1;
	uint8_t int2_status_wom_x: 1;
	uint8_t int2_status_pll_rdy: 1;
} int2_status1_t;

#define EDMP_GPR_0    0x6c
#define EDMP_GPR_1    0x6d
#define EDMP_GPR_2    0x6e
#define EDMP_GPR_3    0x6f
#define EDMP_GPR_CTRL 0x70
typedef struct {
	uint8_t resv_1: 4;
	uint8_t edmp_gpr_ctrl: 4;
} edmp_gpr_ctrl_t;

#define WHO_AM_I 0x72
typedef struct {
	uint8_t who_am_i: 8;
} who_am_i_t;

#define REG_HOST_MSG 0x73
typedef struct {
	uint8_t resv_2: 2;
	uint8_t edmp_on_demand_en: 1;
	uint8_t sa_en: 1;
	uint8_t resv_1: 3;
	uint8_t testopenable: 1;
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
	uint8_t resv_1: 6;
	uint8_t soft_rst: 1;
	uint8_t ireg_done: 1;
} reg_misc2_t;

#ifdef __cplusplus
}
#endif

#endif /*#ifndef _VENICE_A1_CHIPTOP_202212210831_6469331EA6B_EXTERNAL_H_*/
