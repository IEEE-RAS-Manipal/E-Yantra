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

/* Auto-generated by genmsg_cpp from file /home/ch13f_1419/E-Yantra/SS_1302_WS/src/PX4-Autopilot/msg/estimator_event_flags.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/estimator_event_flags.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_estimator_event_flags_fields[] = "\x89 timestamp;\x89 timestamp_sample;\x88 information_event_changes;\x88 warning_event_changes;\x8c gps_checks_passed;\x8c reset_vel_to_gps;\x8c reset_vel_to_flow;\x8c reset_vel_to_vision;\x8c reset_vel_to_zero;\x8c reset_pos_to_last_known;\x8c reset_pos_to_gps;\x8c reset_pos_to_vision;\x8c starting_gps_fusion;\x8c starting_vision_pos_fusion;\x8c starting_vision_vel_fusion;\x8c starting_vision_yaw_fusion;\x8c yaw_aligned_to_imu_gps;\x8c gps_quality_poor;\x8c gps_fusion_timout;\x8c gps_data_stopped;\x8c gps_data_stopped_using_alternate;\x8c height_sensor_timeout;\x8c stopping_navigation;\x8c invalid_accel_bias_cov_reset;\x8c bad_yaw_using_gps_course;\x8c stopping_mag_use;\x8c vision_data_stopped;\x8c emergency_yaw_reset_mag_stopped;\x8c emergency_yaw_reset_gps_yaw_stopped;\x86[7] _padding0;";

ORB_DEFINE(estimator_event_flags, struct estimator_event_flags_s, 49, __orb_estimator_event_flags_fields, static_cast<uint8_t>(ORB_ID::estimator_event_flags));


void print_message(const orb_metadata *meta, const estimator_event_flags_s& message)
{
	if (sizeof(message) != meta->o_size) {
		printf("unexpected message size for %s: %zu != %i\n", meta->o_name, sizeof(message), meta->o_size);
		return;
	}
	orb_print_message_internal(meta, &message, true);
}
