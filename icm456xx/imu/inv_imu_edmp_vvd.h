/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/** @defgroup EDMP EDMP
 *  @brief VVD Model to drive eDMP VVD.
 *  @{
 */

/** @file inv_imu_edmp_vvd.h */

#ifndef _INV_IMU_EDMP_VVD_H_
#define _INV_IMU_EDMP_VVD_H_

#ifdef __cplusplus
extern "C" {
#endif

#define NODE_SIZE_VVD (31) // number of nodes in the tree

// Decision tree is composed of: <threshold_value>, <featureID>, <nextNodeRight>
// decisionTreeThresholds: decision tree thresholds. This is an array of size=nb nodes
// decisionTreeFeatureIDs: array of size=nb nodes: feature index to use for each node.
// decisionTreeNextNodeRight: array of size=nb nodes.
// Start node: index = 0.  If features[featureID] <= decision tree threshold, go to nextNodeLeft, else go to nextNodeRight
// nextNodeLeft = currentNode + 1
// node type can be determined by testing nextNodeRight. if nextNodeRight <= 0, then Node type = leaf, else node type = node
// case 1 (node), threshold_value, featureID, nextNodeRight are set
// case 2 (leaf), threshold_value contains the ID of the class_label, all other values are arbitrary (will not be used)
static const int16_t decisionTreeThresholdsVVD[NODE_SIZE_VVD] = {
	 34,  10, 60, 98, 1, 1, 4, 0,   0, 893, 22, 0,  0, 60, 0, 0, 
	108, 307, 52,  1, 0, 6, 1, 1, 745,  12,  0, 1, 10,  0, 1
};

static const uint8_t decisionTreeFeatureIDsVVD[NODE_SIZE_VVD] = {
	5, 8, 3, 1, 0, 0, 8, 0, 0, 0, 5, 0, 0, 3, 0, 0,
	3, 2, 3, 0, 0, 7, 0, 0, 2, 7, 0, 0, 7, 0, 0
};

static const uint8_t decisionTreeNextNodeRightVVD[NODE_SIZE_VVD] = {
	16,  9,  6, 5, 0,  0, 8, 0, 0, 13, 12, 0,  0, 15, 0, 0,
	24, 21, 20, 0, 0, 23, 0, 0, 28, 27, 0, 0, 30,  0, 0
};

static const uint8_t decisionTreeThresholdsShiftVVD[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_EDMP_VVD_H_ */

/** @} */
