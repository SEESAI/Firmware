**Test ID** | 129
--|--
**PR** | [129](https://github.com/SEESAI/Firmware/pull/129) (port of v1.13 [#52](https://github.com/SEESAI/Firmware/pull/52), [#77](https://github.com/SEESAI/Firmware/pull/77), [#81](https://github.com/SEESAI/Firmware/pull/81))
**Requirement** | PX4 streams `SEES_GPS_RAW` (id 13000) on its MAVLink links: once per `sensor_gps` update on Onboard-mode links and at 1 Hz on Normal-mode links. A receiver built from the same `sees.xml` decodes every field to the values in `sensor_gps`.
**Test lane** | SITL

**Background**
- The companion computer needs PX4's raw GPS data, including course accuracy, heading accuracy and GPS UTC time, which standard MAVLink GPS messages don't carry. v1.13 sent them in a patched `GPS_INPUT`; v1.17 sends the new `SEES_GPS_RAW` from the Sees dialect. See [sees_gps_raw.md](../sees_gps_raw.md).

**Test Method**
- Build and start PX4 SITL with SIH: `make px4_sitl_default sihsim_quadx`.
- In the PX4 shell, list the streams: `mavlink status streams`.
- Run [`scripts/129_sees_gps_raw.py`](./scripts/129_sees_gps_raw.py) on the same machine. It generates the pymavlink `sees` dialect from the `sees.xml` this checkout pins, using that submodule's pymavlink, then listens for 10 s on the Onboard instance's remote port (UDP 14540).
- In the PX4 shell, print the source values: `listener sensor_gps`.

**Pass criteria**

Check | Expected
--|--
`mavlink status streams` | `SEES_GPS_RAW` on every instance: `1.00` on the Normal instance (18570) and `unlimited` on the Onboard instances. Size 93 bytes: 81 bytes of payload plus 12 of MAVLink 2 framing.
Definition, from the script | id 13000, CRC_EXTRA 9, 81-byte payload.
Messages received, from the script | At the `sensor_gps` rate (8 Hz in SIH), `gps_id` 0, script exits 0.
Field values | `lat`/`lon` (degE7), `alt`, `hdop`/`vdop`, `speed_accuracy`, `horiz_accuracy`/`vert_accuracy`, `course_accuracy` and `satellites_visible` equal `latitude_deg`/`longitude_deg`, `altitude_msl_m`, `hdop`/`vdop`, `s_variance_m_s`, `eph`/`epv`, `c_variance_rad` and `satellites_used` in `sensor_gps`.
Unknown values (SIH has no heading and no UTC) | `heading_accuracy` NaN, `yaw` 0, `time_utc_usec` 0, `time_week`/`time_week_ms` 0.

**Results**

2026-11-20, Firmware `vilas/v1.17.0/mavlink`, SEESAI/mavlink `33af200_dev`, SITL built from this branch.

Check | Value | Result
--|--|--
`mavlink status streams` | Normal (18570): `SEES_GPS_RAW 1.00 (1.000) 93`; Onboard (14580, 14280): `SEES_GPS_RAW unlimited 93` | Pass
Definition | id 13000, CRC_EXTRA 9, payload 81 bytes | Pass
Messages received | 80 in 10 s (8.0 Hz), all `gps_id` 0, script exit 0 | Pass
Field values | `lat` 473977394 / `latitude_deg` 47.397739; `lon` 85455945 / 8.545595; `alt` 489.443 / 489.443054; `speed_accuracy` 0.40, `horiz_accuracy` 0.90, `vert_accuracy` 1.78, `course_accuracy` 0.10, `satellites_visible` 10, all equal to `sensor_gps` | Pass
Unknown values | `heading_accuracy` NaN (`sensor_gps.heading` is NaN), `yaw` 0, `time_utc_usec` 0, `time_week`/`time_week_ms` 0 | Pass

Not covered here, and covered by later tests:
- **Decoding by MAVSDK into `Telemetry::GpsInput`:** checked separately against this SITL on the MAVSDK fork, `docs/sees/firmware_v1.17.md` there.
- **Hardware:** finite `heading_accuracy` from a dual-antenna receiver, non-zero `time_utc_usec`, and both GPS instances (`gps_id` 0 and 1) in the right order. See *GPS instance order* in [sees_gps_raw.md](../sees_gps_raw.md#gps-instance-order-gps_id).
