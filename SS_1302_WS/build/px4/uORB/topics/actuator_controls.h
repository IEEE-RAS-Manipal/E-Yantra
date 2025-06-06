/****************************************************************************
 *
 *   Copyright (C) 2013-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/msg/actuator_controls.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define ACTUATOR_CONTROLS_NUM_ACTUATOR_CONTROLS 8
#define ACTUATOR_CONTROLS_NUM_ACTUATOR_CONTROL_GROUPS 4
#define ACTUATOR_CONTROLS_INDEX_ROLL 0
#define ACTUATOR_CONTROLS_INDEX_PITCH 1
#define ACTUATOR_CONTROLS_INDEX_YAW 2
#define ACTUATOR_CONTROLS_INDEX_THROTTLE 3
#define ACTUATOR_CONTROLS_INDEX_FLAPS 4
#define ACTUATOR_CONTROLS_INDEX_SPOILERS 5
#define ACTUATOR_CONTROLS_INDEX_AIRBRAKES 6
#define ACTUATOR_CONTROLS_INDEX_LANDING_GEAR 7
#define ACTUATOR_CONTROLS_INDEX_GIMBAL_SHUTTER 3
#define ACTUATOR_CONTROLS_INDEX_CAMERA_ZOOM 4
#define ACTUATOR_CONTROLS_GROUP_INDEX_ATTITUDE 0
#define ACTUATOR_CONTROLS_GROUP_INDEX_ATTITUDE_ALTERNATE 1
#define ACTUATOR_CONTROLS_GROUP_INDEX_GIMBAL 2
#define ACTUATOR_CONTROLS_GROUP_INDEX_MANUAL_PASSTHROUGH 3
#define ACTUATOR_CONTROLS_GROUP_INDEX_PAYLOAD 6

#endif


#ifdef __cplusplus
struct __EXPORT actuator_controls_s {
#else
struct actuator_controls_s {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float control[8];


#ifdef __cplusplus
	static constexpr uint8_t NUM_ACTUATOR_CONTROLS = 8;
	static constexpr uint8_t NUM_ACTUATOR_CONTROL_GROUPS = 4;
	static constexpr uint8_t INDEX_ROLL = 0;
	static constexpr uint8_t INDEX_PITCH = 1;
	static constexpr uint8_t INDEX_YAW = 2;
	static constexpr uint8_t INDEX_THROTTLE = 3;
	static constexpr uint8_t INDEX_FLAPS = 4;
	static constexpr uint8_t INDEX_SPOILERS = 5;
	static constexpr uint8_t INDEX_AIRBRAKES = 6;
	static constexpr uint8_t INDEX_LANDING_GEAR = 7;
	static constexpr uint8_t INDEX_GIMBAL_SHUTTER = 3;
	static constexpr uint8_t INDEX_CAMERA_ZOOM = 4;
	static constexpr uint8_t GROUP_INDEX_ATTITUDE = 0;
	static constexpr uint8_t GROUP_INDEX_ATTITUDE_ALTERNATE = 1;
	static constexpr uint8_t GROUP_INDEX_GIMBAL = 2;
	static constexpr uint8_t GROUP_INDEX_MANUAL_PASSTHROUGH = 3;
	static constexpr uint8_t GROUP_INDEX_PAYLOAD = 6;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(actuator_controls);
ORB_DECLARE(actuator_controls_0);
ORB_DECLARE(actuator_controls_1);
ORB_DECLARE(actuator_controls_2);
ORB_DECLARE(actuator_controls_3);
ORB_DECLARE(actuator_controls_virtual_fw);
ORB_DECLARE(actuator_controls_virtual_mc);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const actuator_controls_s& message);
#endif
