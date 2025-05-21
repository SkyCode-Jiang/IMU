/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "VSensorImplEISGyr.h"

static struct {
	struct VSensor  vsensor;
	VSensorListener eis_event_listener;
	VSensorListener gyr_listener;
	VSensorDataEIS data;
	int16_t fsync_delay_cnt;
	uint32_t eis_event_timestamp;
} sVSensorImplEISGyr;

static void VSensorImplEISGyr_eisEventHandler(VSensorListener * listener, int event, const void * data)
{
	(void)listener;

	switch (event) {
	case VSENSOR_EVENT_NEW_DATA:
	{
		const VSensorData *eis_event = (const VSensorData *) data;

		sVSensorImplEISGyr.eis_event_timestamp = eis_event->timestamp;
		sVSensorImplEISGyr.fsync_delay_cnt = eis_event->meta_data;
		break;
	}

	default:
		break;
	}
}

static void VSensorImplEISGyr_gyrEventHandler(VSensorListener * listener, int event, const void * data)
{
	switch (event) {
	case VSENSOR_EVENT_NEW_DATA:
	{
		const VSensorDataGyroscope *cal_gyr = (const VSensorDataGyroscope *)data;

		/* if FSYNC event, indicates the delta timestamp between gyro data and preceding FSYNC, -1 otherwise */
		if (cal_gyr->base.timestamp == sVSensorImplEISGyr.eis_event_timestamp)
			sVSensorImplEISGyr.data.delta_ts = sVSensorImplEISGyr.fsync_delay_cnt;
		else
			sVSensorImplEISGyr.data.delta_ts = -1;

		sVSensorImplEISGyr.data.gyr.base.timestamp = cal_gyr->base.timestamp;
		sVSensorImplEISGyr.data.gyr.x = cal_gyr->x;
		sVSensorImplEISGyr.data.gyr.y = cal_gyr->y;
		sVSensorImplEISGyr.data.gyr.z = cal_gyr->z;
		sVSensorImplEISGyr.data.gyr.bx = cal_gyr->bx;
		sVSensorImplEISGyr.data.gyr.by = cal_gyr->by;
		sVSensorImplEISGyr.data.gyr.bz = cal_gyr->bz;

		VSensor_notifyData(&sVSensorImplEISGyr.vsensor, &sVSensorImplEISGyr.data);
		break;
	}

	case VSENSOR_EVENT_NEW_EFFECTIVE_RI:
	{
		listener->vsensor->eri = *(const uint32_t *)data;
		VSensor_notifyEvent(&sVSensorImplEISGyr.vsensor, VSENSOR_EVENT_UPDATERI, &listener->vsensor->eri);
		break;
	}

	default:
		break;
	}
}

static int VSensorImplEISGyr_update(struct VSensor * vsensor, int event, void * data)
{
	switch (event) {
	case VSENSOR_EVENT_SETUP:
		if (VSensorListener_attach(&sVSensorImplEISGyr.eis_event_listener,
				sVSensorImplEISGyr.eis_event_listener.vsensor, &VSensorImplEISGyr_eisEventHandler, 0))
			return -1;
		if (VSensorListener_attach(&sVSensorImplEISGyr.gyr_listener,
				sVSensorImplEISGyr.gyr_listener.vsensor, &VSensorImplEISGyr_gyrEventHandler, 0))
			return -1;

		VSensor_setAttr(vsensor , VSensor_getAttr(sVSensorImplEISGyr.gyr_listener.vsensor));
		break;

	case VSENSOR_EVENT_SUBSCRIBE:
		VSensorListener_enable(&sVSensorImplEISGyr.eis_event_listener);
		VSensorListener_enable(&sVSensorImplEISGyr.gyr_listener);
		break;

	case VSENSOR_EVENT_UNSUBSCRIBE:
		VSensorListener_disable(&sVSensorImplEISGyr.eis_event_listener);
		VSensorListener_disable(&sVSensorImplEISGyr.gyr_listener);
		break;

	case VSENSOR_EVENT_NEW_REQUESTED_RI:
		vsensor->eri = *(const uint32_t *)data;
		VSensorListener_setRi(&sVSensorImplEISGyr.gyr_listener, *(const uint32_t *)data);
		break;

	default:
		return -1;
	}

	return 0;
}

void VSensorImplEISGyr_init(VSensor * eis_event, VSensor * gyr)
{
	memset(&sVSensorImplEISGyr, 0, sizeof(sVSensorImplEISGyr));

	sVSensorImplEISGyr.eis_event_listener.vsensor = eis_event;
	sVSensorImplEISGyr.gyr_listener.vsensor = gyr;

	VSensor_init(&sVSensorImplEISGyr.vsensor, VSensorImplEISGyr_update,
			VSENSOR_TYPE_EIS, sizeof(sVSensorImplEISGyr.data), 0, 0);
}

VSensor * VSensorImplEISGyr_getHandle(void)
{
	return &sVSensorImplEISGyr.vsensor;
}
