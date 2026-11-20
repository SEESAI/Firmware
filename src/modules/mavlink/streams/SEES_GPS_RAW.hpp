/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * SEES: outbound SEES_GPS_RAW stream (Sees dialect, message id 13000).
 *
 * Replaces the v1.13 GPS_INPUT stream (#52, #77, #81), which relied on a
 * modified GPS_INPUT in common.xml. SEES_GPS_RAW carries every GPS_INPUT
 * field plus course_accuracy, heading_accuracy and time_utc_usec.
 * See doc/sees/sees_gps_raw.md in the SEESAI/mavlink repo.
 *
 * Only available when built with CONFIG_MAVLINK_DIALECT="sees".
 */

#ifndef SEES_GPS_RAW_HPP
#define SEES_GPS_RAW_HPP

#include <uORB/topics/sensor_gps.h>

class MavlinkStreamSeesGPSRaw : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSeesGPSRaw(mavlink); }

	static constexpr const char *get_name_static() { return "SEES_GPS_RAW"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SEES_GPS_RAW; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_gps_subs.advertised_count() * (MAVLINK_MSG_ID_SEES_GPS_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamSeesGPSRaw(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<sensor_gps_s> _sensor_gps_subs{ORB_ID::sensor_gps};

	bool send() override
	{
		bool updated = false;

		for (int i = 0; i < _sensor_gps_subs.size(); i++) {
			sensor_gps_s gps;

			if (_sensor_gps_subs[i].update(&gps)) {
				mavlink_sees_gps_raw_t msg{};

				msg.time_usec = gps.timestamp;
				msg.gps_id = i;
				msg.fix_type = gps.fix_type;
				msg.lat = static_cast<int32_t>(round(gps.latitude_deg * 1e7));
				msg.lon = static_cast<int32_t>(round(gps.longitude_deg * 1e7));
				msg.alt = static_cast<float>(gps.altitude_msl_m);
				msg.hdop = gps.hdop;
				msg.vdop = gps.vdop;
				msg.vn = gps.vel_n_m_s;
				msg.ve = gps.vel_e_m_s;
				msg.vd = gps.vel_d_m_s;
				msg.speed_accuracy = gps.s_variance_m_s; // 1-sigma despite the name
				msg.horiz_accuracy = gps.eph;
				msg.vert_accuracy = gps.epv;
				msg.satellites_visible = gps.satellites_used;

				if (!gps.vel_ned_valid) {
					msg.ignore_flags |= GPS_INPUT_IGNORE_FLAG_VEL_HORIZ | GPS_INPUT_IGNORE_FLAG_VEL_VERT;
				}

				// time_week / time_week_ms: not available in sensor_gps, left 0 (as in v1.13)

				// Sees additions (direct copies, see doc/sees/sees_gps_raw.md)
				msg.course_accuracy = gps.c_variance_rad; // 1-sigma despite the name
				msg.heading_accuracy = NAN;
				msg.time_utc_usec = gps.time_utc_usec; // 0 if unknown

				if (PX4_ISFINITE(gps.heading)) {
					if (fabsf(gps.heading) < FLT_EPSILON) {
						msg.yaw = 36000; // Use 36000 for north.

					} else {
						msg.yaw = math::degrees(matrix::wrap_2pi(gps.heading)) * 100.0f; // centidegrees
					}

					if (PX4_ISFINITE(gps.heading_accuracy)) {
						msg.heading_accuracy = gps.heading_accuracy; // rad
					}
				}

				mavlink_msg_sees_gps_raw_send_struct(_mavlink->get_channel(), &msg);

				updated = true;
			}
		}

		return updated;
	}
};

#endif // SEES_GPS_RAW_HPP
