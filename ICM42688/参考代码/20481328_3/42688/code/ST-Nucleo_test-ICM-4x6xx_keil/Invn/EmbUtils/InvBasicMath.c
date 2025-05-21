/*
    Copyright (c) 2014-2015 InvenSense Inc. Portions Copyright (c) 2014-2015 Movea. All rights reserved.

    This software, related documentation and any modifications thereto (collectively "Software") is subject
    to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
    other intellectual property rights laws.

    InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
    and any use, reproduction, disclosure or distribution of the Software without an express license
    agreement from InvenSense is strictly prohibited.
*/

#include "InvBasicMath.h"

#include <limits.h>

unsigned int InvBasicMath_log2u(unsigned int val)
{
	unsigned int ret = UINT_MAX;

	while (val != 0) {
		val >>= 1;
		ret++;
	}

	return ret;
}
