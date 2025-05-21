/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
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

#include "DeviceSmartMotion.h"

#include "Invn/Utils/Message.h"
#include "Invn/VSensorImpl/CModel/VSensorImplCModel.h"
#include "Invn/VSensorImpl/VSensorImplEISGyr.h"

#include <string.h>

/* Forward declaration ********************************************************/

static void init_graph_leaf(inv_device_smart_motion_t * self, struct inv_device_smart_motion_vlistener * ll,
		VSensor * vsensor, int idd_type,
		void (*build_event)(const void *, inv_sensor_event_t *));
static void graph_root_vsensor_init(inv_device_smart_motion_t * self,
		struct inv_device_smart_motion_vsensor * vv, int idd_type, int vsensor_type,
		unsigned vdata_size, void (*build_vsensor_data)(const inv_sensor_event_t *, void *));
static void graph_root_vsensor_feed_data(struct inv_device_smart_motion_vsensor * vv, const inv_sensor_event_t * event);
static void graph_leaf_event_handler(struct VSensorListener * listener, int event, const void * data);
static struct inv_device_smart_motion_vsensor * get_graph_root(inv_device_smart_motion_t * self, int type);

/* Virtual functions table ****************************************************/

static const inv_device_vt_t device_vt = {
	0,
	inv_device_smart_motion_reset,
	inv_device_smart_motion_setup,
	inv_device_smart_motion_cleanup,
	0,
	inv_device_smart_motion_poll,
	inv_device_smart_motion_self_test,
	0,
	inv_device_smart_motion_ping_sensor,
	0,
	inv_device_smart_motion_enable_sensor,
	inv_device_smart_motion_set_sensor_period,
	0,
	inv_device_smart_motion_flush_sensor,
	inv_device_smart_motion_set_sensor_bias,
	inv_device_smart_motion_get_sensor_bias,
	inv_device_smart_motion_set_sensor_mounting_matrix,
	inv_device_smart_motion_get_sensor_data,
	inv_device_smart_motion_set_sensor_config,
	0,
	0,
	0
};


/* Convertion callbacks for root VSensor and leaf VSensorListener **************/

static void build_vdata_fsync_event(const inv_sensor_event_t * ev, void * data)
{
	VSensorData * vdata = (VSensorData *)data;

	vdata->timestamp = (uint32_t)ev->timestamp;
	vdata->meta_data = ev->data.fsync_event.delay_count;
}

static void build_vdata_raw3d_data(const inv_sensor_event_t * ev, void * data)
{
	VSensorDataRaw3d * vdata = (VSensorDataRaw3d *)data;

	vdata->base.timestamp = (uint32_t)ev->timestamp;
	vdata->x = ev->data.raw3d.vect[0];
	vdata->y = ev->data.raw3d.vect[1];
	vdata->z = ev->data.raw3d.vect[2];
	vdata->scale = ev->data.raw3d.fsr;
}


static void build_event_raw_3d(const void * data, inv_sensor_event_t * event)
{
	const VSensorDataRaw3d * vdata = (const VSensorDataRaw3d *)data;

	event->data.raw3d.vect[0] = vdata->x;
	event->data.raw3d.vect[1] = vdata->y;
	event->data.raw3d.vect[2] = vdata->z;
}

static void build_event_3axis_cal(const void * data, inv_sensor_event_t * event)
{
	const VSensorDataCal3d * vdata = (const VSensorDataCal3d *)data;

	event->data.acc.vect[0]       = (float)vdata->x / (1 << 16);
	event->data.acc.vect[1]       = (float)vdata->y / (1 << 16);
	event->data.acc.vect[2]       = (float)vdata->z / (1 << 16);
	event->data.acc.bias[0]       = (float)vdata->bx / (1 << 16);
	event->data.acc.bias[1]       = (float)vdata->by / (1 << 16);
	event->data.acc.bias[2]       = (float)vdata->bz / (1 << 16);
	event->data.acc.accuracy_flag = (uint8_t)(vdata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
}

static void build_event_eis(const void * data, inv_sensor_event_t * event)
{
	const VSensorDataEIS * vdata = (const VSensorDataEIS *)data;

	build_event_3axis_cal(data, event);
	event->data.eis.delta_ts       = vdata->delta_ts;
}

static void build_event_quaternion(const void * data, inv_sensor_event_t * event)
{
	const VSensorDataQuaternion * vdata = (const VSensorDataQuaternion *)data;

	event->data.quaternion.quat[0]       = (float)vdata->w / (1 << 30);
	event->data.quaternion.quat[1]       = (float)vdata->x / (1 << 30);
	event->data.quaternion.quat[2]       = (float)vdata->y / (1 << 30);
	event->data.quaternion.quat[3]       = (float)vdata->z / (1 << 30);
	event->data.quaternion.accuracy      = (float)vdata->accuracy / (1 << 16);
	event->data.quaternion.accuracy_flag = (uint8_t)(vdata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
}

static void build_event_aar(const void * data, inv_sensor_event_t * event)
{
	const VSensorDataAAR * vdata = (const VSensorDataAAR *)data;
	static const struct {
		enum inv_sensor_bac_event a;
		enum VSensorDataAAREvent b;
	} map[] = {
		{ INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN, VSENSOR_DATA_AAR_IN_VEHICLE_START },
		{ INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_END,   VSENSOR_DATA_AAR_IN_VEHICLE_END },
		{ INV_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN,    VSENSOR_DATA_AAR_WALKING_START },
		{ INV_SENSOR_BAC_EVENT_ACT_WALKING_END,      VSENSOR_DATA_AAR_WALKING_END },
		{ INV_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN,    VSENSOR_DATA_AAR_RUNNING_START },
		{ INV_SENSOR_BAC_EVENT_ACT_RUNNING_END,      VSENSOR_DATA_AAR_RUNNING_END },
		{ INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN, VSENSOR_DATA_AAR_ON_BICYCLE_START },
		{ INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_END,   VSENSOR_DATA_AAR_ON_BICYCLE_END },
		{ INV_SENSOR_BAC_EVENT_ACT_TILT_BEGIN,       VSENSOR_DATA_AAR_TILT_START },
		{ INV_SENSOR_BAC_EVENT_ACT_TILT_END,         VSENSOR_DATA_AAR_TILT_END },
		{ INV_SENSOR_BAC_EVENT_ACT_STILL_BEGIN,      VSENSOR_DATA_AAR_STILL_START },
		{ INV_SENSOR_BAC_EVENT_ACT_STILL_END,        VSENSOR_DATA_AAR_STILL_END },
	};
	unsigned i;

	for(i = 0; i < sizeof(map)/sizeof(map[0]); ++i) {
		if(vdata->event == map[i].b) {
			event->data.bac.event = map[i].a;
			return;
		}
	}

	event->data.bac.event = INV_SENSOR_BAC_EVENT_ACT_UNKNOWN;
}

static void build_event_setpc(const void * data, inv_sensor_event_t * event)
{
	const VSensorDataStepCount * vdata = (const VSensorDataStepCount *)data;
	event->data.step.count = vdata->count;
}

static void build_event_oneshot(const void * data, inv_sensor_event_t * event)
{
	(void)data;
	event->data.event = true;
}

static void build_event_orientation(const void * data, inv_sensor_event_t * event)
{
	const VSensorData3dAngles * vdata = (const VSensorData3dAngles *)data;
	event->data.orientation.x             = (float)vdata->x / (1 << 16);
	event->data.orientation.y             = (float)vdata->y / (1 << 16);
	event->data.orientation.z             = (float)vdata->z / (1 << 16);
	event->data.orientation.accuracy_flag = (vdata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
}

static void dummy_callback(const void * data, inv_sensor_event_t * event)
{
	(void)data, (void)event;
}

/******************************************************************************/

/* Initialize root VSensor (ie: VSensor connecting a IDD device to a the SmartMotion VSensor graph)
 */
static void init_graph_roots(inv_device_smart_motion_t * self)
{
	unsigned i;
	const struct {
		unsigned idd_type, vsensor_type, vdata_size;
		void (*build_vsensor_data)(const inv_sensor_event_t *, void *);
		void (*build_event)(const void *, inv_sensor_event_t *);
	} idata[] = {
		{ INV_SENSOR_TYPE_RAW_ACCELEROMETER, VSENSOR_TYPE_RAW_ACCELEROMETER, sizeof(VSensorDataRaw3d),
				build_vdata_raw3d_data, build_event_raw_3d },
		{ INV_SENSOR_TYPE_RAW_GYROSCOPE, VSENSOR_TYPE_RAW_GYROSCOPE, sizeof(VSensorDataRaw3d),
				build_vdata_raw3d_data, build_event_raw_3d },
		{ INV_SENSOR_TYPE_RAW_MAGNETOMETER, VSENSOR_TYPE_RAW_MAGNETOMETER, sizeof(VSensorDataRaw3d),
				build_vdata_raw3d_data, build_event_raw_3d },
		{ INV_SENSOR_TYPE_FSYNC_EVENT, VSENSOR_TYPE_FSYNC_EVENT, sizeof(VSensorData),
				build_vdata_fsync_event, dummy_callback },
	};

	/* do not forget to increase the size of graph_roots[] array if adding new roots */
	assert(sizeof(idata)/sizeof(idata[0]) <= sizeof(self->graph_roots)/sizeof(self->graph_roots[0]));

	for(i = 0; i < sizeof(idata)/sizeof(idata[0]); ++i) {
		graph_root_vsensor_init(self, &self->graph_roots[i].s, idata[i].idd_type,
				idata[i].vsensor_type, idata[i].vdata_size, idata[i].build_vsensor_data);
		init_graph_leaf(self, &self->graph_roots[i].l, &self->graph_roots[i].s.vsensor,
				idata[i].idd_type, idata[i].build_event);
	}
}

/* Helper function to return handle to a VSensor
 * using get_graph_root only if the handle exist
 * otherwise it's return 0
 */
static VSensor * get_graph_root_vsensor(inv_device_smart_motion_t * self, int type)
{
	struct inv_device_smart_motion_vsensor* graph_root;

	graph_root = get_graph_root(self, type);

	if(graph_root)
		return &graph_root->vsensor;
	else
		return 0;
}

/* Initialize CModel VSensor and corresponding leaves
 */
static void init_graph(inv_device_smart_motion_t * self)
{
	unsigned i;
	const struct {
		unsigned idd_type;
		VSensor * vsensor;
		void (*build_event)(const void *, inv_sensor_event_t *);
	} idata[] = {
		{ INV_SENSOR_TYPE_ACCELEROMETER, VSensorImplCModelCalAcc_getHandle(), build_event_3axis_cal },
		{ INV_SENSOR_TYPE_GYROSCOPE, VSensorImplCModelCalGyr_getHandle(), build_event_3axis_cal },
		{ INV_SENSOR_TYPE_UNCAL_GYROSCOPE, VSensorImplCModelUnCalGyr_getHandle(), build_event_3axis_cal },
		{ INV_SENSOR_TYPE_MAGNETOMETER, VSensorImplCModelCalMag_getHandle(), build_event_3axis_cal },
		{ INV_SENSOR_TYPE_UNCAL_MAGNETOMETER, VSensorImplCModelUnCalMag_getHandle(), build_event_3axis_cal },
		{ INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, VSensorImplCModelGRV_getHandle(), build_event_quaternion },
		{ INV_SENSOR_TYPE_ROTATION_VECTOR, VSensorImplCModelRV_getHandle(), build_event_quaternion },
		{ INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR, VSensorImplCModelGeoRV_getHandle(), build_event_quaternion },
		{ INV_SENSOR_TYPE_BAC, VSensorImplCModelAAR_getHandleAAR(), build_event_aar },
		{ INV_SENSOR_TYPE_STEP_COUNTER, VSensorImplCModelAAR_getHandleStepC(), build_event_setpc },
		{ INV_SENSOR_TYPE_STEP_DETECTOR, VSensorImplCModelAAR_getHandleStepD(), build_event_oneshot },
		{ INV_SENSOR_TYPE_SMD, VSensorImplCModelAAR_getHandleSMD(), build_event_oneshot },
		{ INV_SENSOR_TYPE_TILT_DETECTOR, VSensorImplCModelAAR_getHandleTilt(), build_event_oneshot },
		{ INV_SENSOR_TYPE_PICK_UP_GESTURE, VSensorImplCModelPickUp_getHandle(), build_event_oneshot },
		{ INV_SENSOR_TYPE_GRAVITY, VSensorImplCModelGravity_getHandle(), build_event_3axis_cal },
		{ INV_SENSOR_TYPE_LINEAR_ACCELERATION, VSensorImplCModelLinearAcc_getHandle(), build_event_3axis_cal },
		{ INV_SENSOR_TYPE_ORIENTATION, VSensorImplCModelOrientation_getHandle(), build_event_orientation },
		{ INV_SENSOR_TYPE_EIS, VSensorImplEISGyr_getHandle(), build_event_eis },
	};

	VSensorImplCModelCalAcc_init(get_graph_root_vsensor(self, INV_SENSOR_TYPE_RAW_ACCELEROMETER));
	VSensorImplCModelCalGyr_init(get_graph_root_vsensor(self, INV_SENSOR_TYPE_RAW_GYROSCOPE));
	VSensorImplCModelCalMag_init(get_graph_root_vsensor(self, INV_SENSOR_TYPE_RAW_MAGNETOMETER));
	VSensorImplCModelGRV_init(VSensorImplCModelCalGyr_getHandle(), VSensorImplCModelCalAcc_getHandle());
	VSensorImplCModelRV_init(VSensorImplCModelCalAcc_getHandle(), VSensorImplCModelCalGyr_getHandle(),
			VSensorImplCModelCalMag_getHandle());
	VSensorImplCModelGeoRV_init(VSensorImplCModelCalAcc_getHandle(), VSensorImplCModelCalMag_getHandle());
	VSensorImplCModelAAR_init(VSensorImplCModelCalAcc_getHandle());
	VSensorImplCModelPickUp_init(VSensorImplCModelCalAcc_getHandle());
	VSensorImplCModelGravity_init(VSensorImplCModelGRV_getHandle());
	VSensorImplCModelLinearAcc_init(VSensorImplCModelGravity_getHandle(), VSensorImplCModelCalAcc_getHandle());
	VSensorImplCModelOrientation_init(VSensorImplCModelRV_getHandle());

	VSensorImplEISGyr_init(get_graph_root_vsensor(self, INV_SENSOR_TYPE_FSYNC_EVENT), VSensorImplCModelCalGyr_getHandle());

	/* do not forget to increase the size of graph_leaves[] array if adding new leaf */
	assert(sizeof(idata)/sizeof(idata[0]) <= sizeof(self->graph_leaves)/sizeof(self->graph_leaves[0]));

	for(i = 0; i < sizeof(idata)/sizeof(idata[0]); ++i) {
		init_graph_leaf(self, &self->graph_leaves[i].l, idata[i].vsensor,
				idata[i].idd_type, idata[i].build_event);
	}

}

/******************************************************************************/

/* Ping sensor from underlying HW device
 * Ping only if HW sensor is not already set as available
 * If available, update hw_sensor_available_mask
 */
static int ping_hw_sensor(inv_device_smart_motion_t * self, int sensor)
{
	/* Sensor type is not set in mask */
	if((self->hw_sensor_available_mask & (1ULL << (unsigned)sensor)) == 0) {
		/* Ping HW sensor */
		const int rc = inv_device_ping_sensor(self->hw_device, sensor);

		if(rc == 0) {
			/* Set sensor as available at HW level */
			self->hw_sensor_available_mask |= (1ULL << sensor);
		}

		return rc;
	}
	else {
		/* Sensor already set: ping OK */
		return 0;
	}
}


/* Return sensor available from underlying device by looking at in-cache mask
 */
static inv_bool_t is_hw_sensor_available(inv_device_smart_motion_t * self, int sensor)
{
	if(!INV_SENSOR_IS_WU(sensor) && INV_SENSOR_IS_VALID(sensor)
			&& (self->hw_sensor_available_mask & (1ULL << sensor)) != 0) {
		return true;
	}

	return false;
}


/* Helper function to return handle to a leaf (struct inv_device_smart_motion_vlistener object)
 * according to its IDD sensor type
 */
static struct inv_device_smart_motion_vlistener * get_graph_leaf(inv_device_smart_motion_t * self, int type)
{
	if(!INV_SENSOR_IS_WU(type) && INV_SENSOR_IS_VALID(type)) {
		struct InvList *cur = InvList_head(&self->leaves_list);

		while (cur != 0) {
			struct inv_device_smart_motion_vlistener * ll = INVLIST_GET(cur, struct inv_device_smart_motion_vlistener, node);

			if(ll->idd_type == type) {
				return ll;
			}

			cur = InvList_next(cur);
		}
	}

	return 0;
}


/* Helper function to return handle to a root (struct inv_device_smart_motion_vsensor object)
 * according to its IDD sensor type
 */
static struct inv_device_smart_motion_vsensor * get_graph_root(inv_device_smart_motion_t * self, int type)
{
	unsigned i;

	for(i = 0; i < sizeof(self->graph_roots)/sizeof(self->graph_roots[0]); ++i) {
		struct inv_device_smart_motion_vsensor * cur = &self->graph_roots[i].s;

		if(cur->idd_type == type)
			return cur;
	}

	return 0;
}


/* Helper function to initialize struct inv_device_cmodel_vlistener object
 */
static void init_graph_leaf(inv_device_smart_motion_t * self, struct inv_device_smart_motion_vlistener * ll,
		VSensor * vsensor, int idd_type,
		void (*build_event)(const void *, inv_sensor_event_t *))
{
	if(VSensorListener_attach(&ll->vlistener, vsensor, graph_leaf_event_handler, self) == 0) {
		ll->idd_type    = idd_type;
		ll->build_event = build_event;
		InvList_add(&self->leaves_list, &ll->node);

		/* set default RI to 200ms (5Hz) */
		VSensorListener_setRi(&ll->vlistener, 200000 /*us*/);
	}
}


/* Internal VSensorListener event handler for leaves
 * Will convert a VSensorData to a inv_sensor_event_t and notify IDD client
 */
static void graph_leaf_event_handler(struct VSensorListener * listener,
		int event, const void * data)
{
	struct inv_device_smart_motion_vlistener * ll = (struct inv_device_smart_motion_vlistener *)listener;
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)ll->vlistener.arg;
	inv_sensor_event_t ev;

	memset(&ev, 0, sizeof(ev));
	ev.sensor = ll->idd_type;

	if(event == VSENSOR_EVENT_NEW_DATA) {
		ev.status    = INV_SENSOR_STATUS_DATA_UPDATED;
		ev.timestamp = ((const VSensorData *)data)->timestamp;
		ll->build_event(data, &ev);
	}
	else if(event == VSENSOR_EVENT_FLUSH_COMPLETE) {
		ev.status    = INV_SENSOR_STATUS_FLUSH_COMPLETE;
		ev.timestamp = 0;
	}
	else {
		/* Do not propagate other events */
		return;
	}
	
	/* Notify IDD client */
	inv_sensor_listener_notify(self->base.listener, &ev);
}


/* Internal IDD listener sensor event
 * Will catch sensor events comming from underlying device and either feed CModel
 * graph or forward events to IDD clent
 */
static void hw_device_event_cb(const inv_sensor_event_t * event, void * context)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	struct inv_device_smart_motion_vsensor * vv = get_graph_root(self, event->sensor);

	if(vv) {
		/* Feed VSensor */
		graph_root_vsensor_feed_data(vv, event);
	}
	else {
		/* Forward event directly to IDD client */
		inv_sensor_listener_notify(self->base.listener, event);
	}
}

/* IDD function implementation ************************************************/

void inv_device_smart_motion_init(inv_device_smart_motion_t * self, inv_device_t * hw_device)
{
	memset(self, 0, sizeof(*self));
	self->base.instance = self;
	self->base.vt       = &device_vt;
	self->base.listener = hw_device->listener;
	self->hw_device     = hw_device;

	inv_sensor_listener_init(&self->private_idd_listener, hw_device_event_cb, self);
	hw_device->listener = &self->private_idd_listener;
}

int inv_device_smart_motion_poll(void * context)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	
	return inv_device_poll(self->hw_device);
}

int inv_device_smart_motion_self_test(void * context, int sensor)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;

	if(is_hw_sensor_available(self, sensor)) {
		return inv_device_self_test(self->hw_device, sensor);
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_smart_motion_reset(void * context)
{
	/* Re-init the device */
	return inv_device_setup(context);
}


int inv_device_smart_motion_setup(void * context)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	int sensor;

	/* Reset graph */
	self->hw_sensor_available_mask = 0;
	InvList_init(&self->leaves_list);

	/* Init graph */
	init_graph_roots(self);
	init_graph(self);

	/* Ping all hw sensors and remove emulated sensor that are available at hw level */
	for(sensor = INV_SENSOR_TYPE_RESERVED + 1; 
			sensor < INV_SENSOR_TYPE_MAX; ++sensor) {

		const int rc = ping_hw_sensor(self, sensor);

		if(rc == 0) {
			struct inv_device_smart_motion_vlistener * ll = get_graph_leaf(self, sensor);

			/* Do not remove listeners needed for roots */
			if(ll && !get_graph_root(self, sensor)) {
				InvList_remove(&self->leaves_list, &ll->node);
			}
		}
	}

	return 0;
}

int inv_device_smart_motion_cleanup(void * context)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;

	(void)self;

	/* Nothing to do for now...*/
	
	return 0;
}

int inv_device_smart_motion_ping_sensor(void * context, int sensor)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;

	if(get_graph_leaf(self, sensor) == 0 && is_hw_sensor_available(self, sensor) == 0) {
		return INV_ERROR;
	}

	return 0;
}

int inv_device_smart_motion_enable_sensor(void * context, int sensor, inv_bool_t en)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	struct inv_device_smart_motion_vlistener * ll = get_graph_leaf(self, sensor);

	if(ll) {
		if(en) {
			VSensorListener_enable(&ll->vlistener);
		}
		else {
			VSensorListener_disable(&ll->vlistener);
		}
		return 0;
	}
	else if(is_hw_sensor_available(self, sensor)) {
		return inv_device_enable_sensor(self->hw_device, sensor, en);
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_smart_motion_set_sensor_period(void * context,
		int sensor, uint32_t period)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	struct inv_device_smart_motion_vlistener * ll = get_graph_leaf(self, sensor);

	if(ll) {
		VSensorListener_setRi(&ll->vlistener, period);
		return 0;
	}
	else if(is_hw_sensor_available(self, sensor)) {
		return inv_device_set_sensor_period(self->hw_device, sensor, period);
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_smart_motion_flush_sensor(void * context, int sensor)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	struct inv_device_smart_motion_vlistener * ll = get_graph_leaf(self, sensor);

	if(ll) {
		VSensor_update(VSensorListener_getVSensor(&ll->vlistener),
				VSENSOR_EVENT_FLUSH_DATA, 0);

		return 0;
	}
	else if(is_hw_sensor_available(self, sensor)) {
		return inv_device_flush_sensor(self->hw_device, sensor);
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_smart_motion_set_sensor_bias(void * context, int sensor,
		const float bias[3])
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	struct inv_device_smart_motion_vlistener * ll = get_graph_leaf(self, sensor);

	if(ll) {
		VSensorConfigOffset cfg;

		cfg.base.type = VSENSOR_CONFIG_TYPE_OFFSET;
		cfg.vect[0] = (intq16_t)(bias[0] * (1 << 16));
		cfg.vect[1] = (intq16_t)(bias[1] * (1 << 16));
		cfg.vect[2] = (intq16_t)(bias[2] * (1 << 16));

		VSensor_update(VSensorListener_getVSensor(&ll->vlistener),
				VSENSOR_EVENT_SET_CONFIG, &cfg);

		return 0;
	}
	else if(is_hw_sensor_available(self, sensor)) {
		return inv_device_set_sensor_bias(self->hw_device, sensor, bias);
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_smart_motion_get_sensor_bias(void * context, int sensor,
		float bias[3])
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	struct inv_device_smart_motion_vlistener * ll = get_graph_leaf(self, sensor);

	if(ll) {
		VSensorConfigOffset cfg;
	
		cfg.base.type = VSENSOR_CONFIG_TYPE_OFFSET;

		if(VSensor_update(VSensorListener_getVSensor(&ll->vlistener),
				VSENSOR_EVENT_GET_CONFIG, &cfg) == 0) {

			bias[0] = (float)cfg.vect[0] / (1 << 16);
			bias[1] = (float)cfg.vect[1] / (1 << 16);
			bias[2] = (float)cfg.vect[2] / (1 << 16);

			return 0;
		}
	}
	else if(is_hw_sensor_available(self, sensor)) {
		return inv_device_get_sensor_bias(self->hw_device, sensor, bias);
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_smart_motion_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9])
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	struct inv_device_smart_motion_vlistener * ll = get_graph_leaf(self, sensor);

	if(ll) {
		int i;
		VSensorConfigReferenceFrame cfg;

		cfg.base.type = VSENSOR_CONFIG_TYPE_REFERANCE_FRAME;
		for(i = 0; i < 9; ++i) {
			cfg.matrix[i] = (intq30_t)(matrix[i] * (1 << 30));
		}

		VSensor_update(VSensorListener_getVSensor(&ll->vlistener),
				VSENSOR_EVENT_SET_CONFIG, &cfg);

		return 0;
	} 
	else if(is_hw_sensor_available(self, sensor)) {
		return inv_device_set_sensor_mounting_matrix(self->hw_device, sensor, matrix);
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_smart_motion_get_sensor_data(void * context, int sensor,
		inv_sensor_event_t * event)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	struct inv_device_smart_motion_vlistener * ll = get_graph_leaf(self, sensor);

	memset(event, 0, sizeof(*event));

	if(ll) {
		VSensorDataAny data;

		if(VSensor_update(VSensorListener_getVSensor(&ll->vlistener),
				VSENSOR_EVENT_GET_DATA, &data) == 0) {

			ll->build_event(&data, event);
			event->timestamp = data.base.timestamp;
			event->sensor    = sensor;
			event->status    = INV_SENSOR_STATUS_POLLED_DATA;

			return 0;
		}
	}
	else if(is_hw_sensor_available(self, sensor)) {
		return inv_device_get_sensor_data(self->hw_device, sensor, event);
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_smart_motion_set_sensor_config(void * context, int sensor, int setting,
		const void * value, unsigned size)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)context;
	struct inv_device_smart_motion_vlistener * ll = get_graph_leaf(self, sensor);
	
	if(ll) {
		//check if it's to configure sensitivity
		if(setting == INV_SMARTMOTION_CONFIG_SENSITIVITY) {
			VSensorImplCModelConfigSensitivity cfg;

			cfg.base.type = VSENSOR_IMPL_CMODEL_CONFIG_TYPE_SENSITIVITY;
			cfg.sensitivity = *(const uint32_t*) value;

			if(VSensor_update(VSensorListener_getVSensor(&ll->vlistener), VSENSOR_EVENT_SET_CONFIG, &cfg) != 0)
				return INV_ERROR;
		}
		else if(setting == INV_SMARTMOTION_RV_THRESH_GYR_STOP_CONVERG) {
			VSensorImplCModelConfigRvThreshGyrStop cfg;
			cfg.base.type = VSENSOR_IMPL_CMODEL_CONFIG_TYPE_RV_THRESHOLD_GYR_STOP_CONVERGENCE;
			cfg.rv_thresh_gyr_stop = *(const int32_t*) value;

			if(VSensor_update(VSensorListener_getVSensor(&ll->vlistener), VSENSOR_EVENT_SET_CONFIG, &cfg) != 0)
				return INV_ERROR;
		}
		else if(setting == INV_SMARTMOTION_RV_THRESH_CAL_GYR) {
			VSensorImplCModelConfigGyrCal cfg;
			const int32_t * i32_val = (const int32_t *)value;
			cfg.base.type = VSENSOR_IMPL_CMODEL_CONFIG_TYPE_CAL_GYR;
			cfg.sampleNum_log2     = i32_val[0];
			cfg.data_diff_thrsh    = i32_val[1];
			cfg.fnm_sampleNum_log2 = i32_val[2];
			cfg.fnm_moment_thrsh   = i32_val[3];
			cfg.fnm_data_abs_thrsh = i32_val[4];

			if(VSensor_update(VSensorListener_getVSensor(&ll->vlistener), VSENSOR_EVENT_SET_CONFIG, &cfg) != 0)
				return INV_ERROR;
		}
		else if(setting == INV_SENSOR_CONFIG_GAIN) {
			int i;
			VSensorConfigGain cfg;
			const inv_sensor_config_gain_t * gain = (const inv_sensor_config_gain_t *)value;

			cfg.base.type = VSENSOR_CONFIG_TYPE_GAIN;

			for(i = 0; i < 9; ++i) {
				cfg.matrix[i] = (int32_t)(gain->gain[i] * (1L << 30));
			}

			if(VSensor_update(VSensorListener_getVSensor(&ll->vlistener), VSENSOR_EVENT_SET_CONFIG, &cfg) != 0)
				return INV_ERROR;
		}
		return 0;
	}
	else if(is_hw_sensor_available(self, sensor)) {
		return inv_device_set_sensor_config(self->hw_device, sensor, setting, value, size);
	}
	return INV_ERROR_BAD_ARG;
}

/* VSensor implementation for roots *******************************************/

static int graph_root_vsensor_update(struct VSensor * vsensor, int event, void * data);

/* Helper function to initialize a root VSensor (struct inv_device_cmodel_vsensor *)
 */
static void graph_root_vsensor_init(inv_device_smart_motion_t * self,
		struct inv_device_smart_motion_vsensor * vv, int idd_type, int vsensor_type,
		unsigned vdata_size, void (*build_vsensor_data)(const inv_sensor_event_t *, void *))
{
	VSensor_init(&vv->vsensor, graph_root_vsensor_update, vsensor_type, vdata_size,
			0 /* use default attr */, self /* arg */);
	vv->build_vsensor_data = build_vsensor_data;
	vv->idd_type = idd_type;
}

/* Function to feed update a root VSensor data and feed the 'CModel' graph
 * Will create corresponding VSensorData from a inv_sensor_event_t
 * and notify listeners of the root VSensor
 */
static void graph_root_vsensor_feed_data(struct inv_device_smart_motion_vsensor * vv, const inv_sensor_event_t * event)
{
	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {
		VSensorDataAny data;

		data.base.timestamp = (uint32_t)(event->timestamp & UINT32_MAX);
		vv->build_vsensor_data(event, &data);
		VSensor_notifyData(&vv->vsensor, &data);
	}
	else if(event->status == INV_SENSOR_STATUS_FLUSH_COMPLETE) {
		VSensor_notifyEvent(&vv->vsensor, VSENSOR_EVENT_FLUSH_COMPLETE, 0);
	}
}

/* Helper function called upon VSENSOR_EVENT_GET_DATA event
 * Read last sensor data from underlying device and convert it to VSensorData before returning
 */
static int graph_root_vsensor_get_data(struct inv_device_smart_motion_vsensor * vv, void * data)
{
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)vv->vsensor.arg;
	inv_sensor_event_t event;

	if(inv_device_get_sensor_data(self->hw_device, vv->idd_type, &event) == 0) {
		VSensorDataAny * vdata = (VSensorDataAny *)data;

		vdata->base.timestamp = (uint32_t)(event.timestamp & UINT32_MAX);
		vv->build_vsensor_data(&event, vdata);

		return 0;
	}

	return -1;
}

/* Helper function called upon VSENSOR_EVENT_GET_CONFIG event
 * Call corresponding configuration method from underlying device
 */
static int graph_root_vsensor_get_config(struct inv_device_smart_motion_vsensor * vv, void * data)
{
	const VSensorConfig * cfg = (const VSensorConfig *)data;
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)vv->vsensor.arg;

	if(cfg->type == VSENSOR_CONFIG_TYPE_OFFSET) {
		VSensorConfigOffset * offset = (VSensorConfigOffset *)data;
		float bias[3];

		if(inv_device_get_sensor_bias(self->hw_device, vv->idd_type, bias) == 0) {
			offset->vect[0] = (int32_t)(bias[0] * (1 << 16));
			offset->vect[1] = (int32_t)(bias[1] * (1 << 16));
			offset->vect[2] = (int32_t)(bias[2] * (1 << 16));

			return 0;
		}
	}

	return -1;
}

/* Helper function called upon VSENSOR_EVENT_SET_CONFIG event
 * Convert config data and call corresponding configuration method from underlying device
 */
static int graph_root_vsensor_set_config(struct inv_device_smart_motion_vsensor * vv, const void * data)
{
	const VSensorConfig * cfg = (const VSensorConfig *)data;
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)vv->vsensor.arg;

	if(cfg->type == VSENSOR_CONFIG_TYPE_REFERANCE_FRAME) {
		const VSensorConfigReferenceFrame * refframe = (const VSensorConfigReferenceFrame *)data;
		float matrix[9];
		unsigned i;

		for(i = 0; i < 9; ++i) {
			matrix[i] = (float)refframe->matrix[i] / (1 << 30);
		}

		return inv_device_set_sensor_mounting_matrix(self->hw_device, vv->idd_type, matrix);
	}
	else if(cfg->type == VSENSOR_CONFIG_TYPE_OFFSET) {
		const VSensorConfigOffset * offset = (const VSensorConfigOffset *)data;
		float bias[3];
		
		bias[0] = (float)offset->vect[0] / (1 << 16);
		bias[1] = (float)offset->vect[1] / (1 << 16);
		bias[2] = (float)offset->vect[2] / (1 << 16);
		
		return inv_device_set_sensor_bias(self->hw_device, vv->idd_type, bias);
	}

	return -1;
}

/* Main update function for VSensor implementation
 */
static int graph_root_vsensor_update(struct VSensor * vsensor, int event, void * data)
{
	struct inv_device_smart_motion_vsensor * vv = (struct inv_device_smart_motion_vsensor *)vsensor;
	inv_device_smart_motion_t * self = (inv_device_smart_motion_t *)vv->vsensor.arg;

	switch(event) {
	case VSENSOR_EVENT_SETUP:
		return ping_hw_sensor(self, vv->idd_type);
	case VSENSOR_EVENT_SUBSCRIBE:
		return inv_device_enable_sensor(self->hw_device, vv->idd_type, true);
	case VSENSOR_EVENT_UNSUBSCRIBE:
		return inv_device_enable_sensor(self->hw_device, vv->idd_type, false);
	case VSENSOR_EVENT_NEW_REQUESTED_RI:
		return inv_device_set_sensor_period_us(self->hw_device, vv->idd_type, *(const uint32_t *)data);
	case VSENSOR_EVENT_NEW_REQUESTED_MRL:
		return inv_device_set_sensor_timeout_us(self->hw_device, vv->idd_type, *(const uint32_t *)data);
	case VSENSOR_EVENT_FLUSH_DATA:
		return inv_device_flush_sensor(self->hw_device, vv->idd_type);
	case VSENSOR_EVENT_GET_DATA:
		return graph_root_vsensor_get_data(vv, data);
	case VSENSOR_EVENT_SET_CONFIG:
		return graph_root_vsensor_set_config(vv, data);
	case VSENSOR_EVENT_GET_CONFIG:
		return graph_root_vsensor_get_config(vv, data);
	}

	return -1;
}
