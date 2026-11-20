# `SEES_GPS_RAW` MAVLink Stream

| | |
|---|---|
| **Status** | Port of [#52](https://github.com/SEESAI/Firmware/pull/52), [#77](https://github.com/SEESAI/Firmware/pull/77) and [#81](https://github.com/SEESAI/Firmware/pull/81) for v1.17, in [#129](https://github.com/SEESAI/Firmware/pull/129). Tested in SITL ([test 129](tests/test_129.md)). The consumers (MAVSDK, SeesInterface2, QGroundControl) follow; see [Rollout across repos](#rollout-across-repos). |
| **Code** | [SEES_GPS_RAW.hpp](../../src/modules/mavlink/streams/SEES_GPS_RAW.hpp), [mavlink_messages.cpp](../../src/modules/mavlink/mavlink_messages.cpp), [mavlink_main.cpp](../../src/modules/mavlink/mavlink_main.cpp) (`configure_streams_to_default`), board `default.px4board` files |
| **Message definition** | `message_definitions/v1.0/sees.xml` in the [SEESAI/mavlink](https://github.com/SEESAI/mavlink) submodule, branch `33af200_dev`. Its [doc](https://github.com/SEESAI/mavlink/blob/33af200_dev/doc/sees/sees_gps_raw.md) covers the message's design, field semantics and wire details |
| **Test** | [tests/test_129.md](tests/test_129.md) |
| **Migration plan** | [Update MAVlink and introduce GPS fields for SI2](migration_plan_v1.17.md#update-mavlink-and-introduce-gps-fields-for-si2) |

## Summary

PX4 sends the companion computer its raw GPS data, including three fields that standard MAVLink GPS messages don't carry:

- course accuracy
- heading accuracy
- GPS UTC time

In v1.13 this was a `GPS_INPUT` stream, with these fields added to `GPS_INPUT` by patching `common.xml`. In v1.17 the stream sends a new message, `SEES_GPS_RAW` (ID 13000), defined in a Sees MAVLink dialect, `sees.xml`, so `common.xml` stays unmodified.

The data flows **from PX4 to the companion**; PX4 doesn't receive `SEES_GPS_RAW`. The message has the same fields as the v1.13 `GPS_INPUT`. It's named after `GPS_RAW_INT`, the standard autopilot → companion GPS message, because `GPS_INPUT` is defined for the opposite direction.

## Why

### A new message instead of patching `GPS_INPUT`

The v1.13 patch put the Sees fields among `GPS_INPUT`'s extension fields, ahead of upstream's own `yaw` extension:

- standard MAVLink code read `course_accuracy` where it expected `yaw`;
- extension fields are outside the message's CRC, so a mismatch between sender and receiver was never detected, just misread;
- every mavlink update meant carrying and re-checking the `common.xml` patch.

`SEES_GPS_RAW` is a separate message with its own CRC. A receiver built with a different definition drops it instead of misreading it, and a mavlink update needs no Sees patch.

### The `development` base dialect

`sees.xml` includes `development.xml`, which includes `common.xml`. PX4 generates one main dialect per board, so a board must be built with `sees` to have `SEES_GPS_RAW`; it then also gets every development message.

- SITL and upstream's main boards (fmu-v5x, fmu-v6x) already build with `development`, so the Cubes now have the same message set as SITL.
- Development messages turn on newer PX4 features, such as `AVAILABLE_MODES` and `CURRENT_MODE` for external modes.
- The cost: development messages can change between mavlink updates, and they use about 2.5 KB more flash on each Cube (of 1920 KB):

  | Board | `common` | `sees` | Difference |
  |---|---|---|---|
  | CubeOrange | 1,929,496 B (98.14%) | 1,932,032 B (98.27%) | +2,536 B |
  | CubeOrangePlus | 1,934,760 B (98.41%) | 1,937,280 B (98.54%) | +2,520 B |

## What was done

### Submodule and dialect

- `src/modules/mavlink/mavlink` points at SEESAI/mavlink: `.gitmodules` tracks `33af200_dev` (upstream `33af200d` plus `sees.xml`), and the submodule pins a commit on it.
- `CONFIG_MAVLINK_DIALECT="sees"` is set for `cubepilot/cubeorange` and `cubepilot/cubeorangeplus` (previously the default, `common`) and `px4/sitl` (previously `development`).

### Stream

[`SEES_GPS_RAW.hpp`](../../src/modules/mavlink/streams/SEES_GPS_RAW.hpp) is the final v1.13 `GPS_INPUT.hpp` (#52, #77, #81), updated for v1.17's `sensor_gps` fields. It subscribes to every `sensor_gps` instance, and sends one message per update, with `gps_id` set to the instance index.

Its include in `mavlink_messages.cpp` is guarded by `#if defined(MAVLINK_MSG_ID_SEES_GPS_RAW)`, the same way upstream guards `GNSS_INTEGRITY`, so boards built with other dialects compile without it.

Default rates, set in `Mavlink::configure_streams_to_default()`, are the same as v1.13's `GPS_INPUT`:

| MAVLink mode | `MAV_X_MODE` | Rate | Typical link |
|---|---|---|---|
| Normal | 0 | 1 Hz | GCS telemetry radio |
| Onboard | 2 | every `sensor_gps` update | Companion computer |
| Config | 5 | every `sensor_gps` update | USB |

### Field mapping

Checked field by field against the v1.13 stream (#52, with #77/#81 adding the accuracy fields):

| `SEES_GPS_RAW` field | Source in `sensor_gps` | v1.13 `GPS_INPUT` |
|---|---|---|
| `time_usec` | `timestamp` (time since boot) | Same |
| `gps_id` | uORB instance index; see [GPS instance order](#gps-instance-order-gps_id) | Same |
| `fix_type` | `fix_type` | Same |
| `lat`, `lon` | `latitude_deg`, `longitude_deg`, converted to degE7 | Same units; v1.13's `sensor_gps` already held degE7 |
| `alt` | `altitude_msl_m` (m, MSL) | Same units; v1.13 converted from mm |
| `hdop`, `vdop` | `hdop`, `vdop` | Same |
| `vn`, `ve`, `vd` | `vel_n_m_s`, `vel_e_m_s`, `vel_d_m_s` | Same |
| `horiz_accuracy`, `vert_accuracy` | `eph`, `epv` | Same |
| `speed_accuracy` | `s_variance_m_s` (1-sigma despite the name) | Same (#77/#81) |
| `course_accuracy` | `c_variance_rad` (rad, 1-sigma despite the name) | Same (#77/#81) |
| `heading_accuracy` | `heading_accuracy` (rad) when `heading` and `heading_accuracy` are both finite, else **NaN** | v1.13 sent **0** when unknown (#77/#81) |
| `yaw` | `heading`, in cdeg; 36000 for north; 0 when `heading` is NaN | Same |
| `time_utc_usec` | `time_utc_usec`; 0 until the receiver has UTC | Same (#52) |
| `satellites_visible` | `satellites_used` | **New**: v1.13 left it 0 |
| `ignore_flags` | `VEL_HORIZ \| VEL_VERT` when `vel_ned_valid` is false, else 0 | **New**: v1.13 left it 0 |
| `time_week_ms`, `time_week` | Always 0: `sensor_gps` has no GPS week time | Same |

No uORB `.msg` changes were needed.

### GPS instance order (`gps_id`)

`gps_id` is the `sensor_gps` uORB instance index, which depends on the order in which the GPS drivers first publish. SeesInterface2 treats `gps_id` 0 as the primary receiver (the rover): it takes the GPS time sync and yaw from it, and fills its primary and secondary GPS data from `gps_id` 0 and 1.

In v1.13 the order was forced. The moving base wasn't published until the rover had claimed `sensor_gps` instance 0 (#46, #50, #69, #94, #96). Those changes aren't ported to v1.17 yet; see [Dual-CAN-GPS node-ID ordering / Rover-ID](migration_plan_v1.17.md#dual-can-gps-node-id-ordering--rover-id) in the migration plan.

**Before SeesInterface2 uses `SEES_GPS_RAW` with two receivers,** that order must be confirmed or enforced again. Otherwise `gps_id` 0 may be the moving base. Checking which `gps_id` has yaw doesn't work as a fallback, because yaw drops out and isn't there at start-up.

## Using it

### Companion link

The stream is on by default on any MAVLink instance in Onboard mode (`MAV_X_MODE = 2`). To change the rate on a running system, from the `nsh` console:

```sh
mavlink stream -d /dev/ttyS1 -s SEES_GPS_RAW -r 5    # serial instance, 5 Hz
mavlink stream -u 14580 -s SEES_GPS_RAW -r -1         # UDP instance (local port), back to default
```

For a permanent change, add the line to the board's `extras.txt` or change `configure_streams_to_default()`.

### Receiving it

A receiver must use a MAVLink library generated from the **same** `sees.xml` that PX4 was built with. Otherwise every `SEES_GPS_RAW` fails its CRC check (CRC_EXTRA 9) and is dropped. For example, from the submodule:

```sh
cd src/modules/mavlink/mavlink
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=out message_definitions/v1.0/sees.xml
```

Values with no data:

- `heading_accuracy` is NaN without a dual-antenna heading;
- `time_utc_usec` is 0 until the receiver has UTC;
- `time_week` and `time_week_ms` are always 0.

### Another board

Add `CONFIG_MAVLINK_DIALECT="sees"` to the board's `.px4board`, and check its flash usage; the development messages add about 2.5 KB.

## Changing the message

The definition lives in SEESAI/mavlink, branch `33af200_dev`:

1. Change `sees.xml` in a PR into `33af200_dev`, using IDs in the reserved 13000–13999 range. Run the checks in the mavlink doc.
2. After it's merged, re-pin every consumer to the commit on `33af200_dev`:
   - Firmware's submodule (`git submodule update --remote src/modules/mavlink/mavlink`), with any stream changes;
   - MAVSDK's `MAVLINK_GIT_HASH`;
   - SeesInterface2's `mavlink-seesai` and `mavsdk` vcpkg ports;
   - QGroundControl's MAVLink headers.

   Changing any field changes the CRC, so a consumer built from the old definition drops every message until it's rebuilt.

Pin only commits on a branch. A squash or rebase, including a squash merge of a PR, creates new commits, so pin after the merge.

## Rollout across repos

Firmware v1.17 is the reference; every consumer is made compatible with it.

| Step | Repo, branch | Status |
|---|---|---|
| 1. Message | SEESAI/mavlink `33af200_dev` | Done ([mavlink#3](https://github.com/SEESAI/mavlink/pull/3)) |
| 2. Stream | SEESAI/Firmware `vilas/v1.17.0/mavlink` → `v1.17.0-dev` | This PR ([#129](https://github.com/SEESAI/Firmware/pull/129)) |
| 3. Decode | SEESAI/MAVSDK `vilas/feat/firmware_v1.17` → `v3.7.0_dev` | In progress ([MAVSDK#25](https://github.com/SEESAI/MAVSDK/pull/25), draft) |
| 4. Use | SEESAI/SeesInterface2 `vilas/feat/firmware_v1.17` | In progress |
| 5. Display | SEESAI/qgroundcontrol | To do |
| 6. Hardware test and release | all | To do |

**Decisions**

- **MAVSDK v3.7.0**, the MAVSDK release PX4 v1.17 tests against (`test/mavsdk_tests/MAVSDK_VERSION`), built against the same SEESAI/mavlink commit as Firmware. It decodes `SEES_GPS_RAW` into the existing Sees `Telemetry::GpsInput`, so SeesInterface2's GPS code stays the same.
- **QGroundControl must show `SEES_GPS_RAW`,** as it showed the v1.13 `GPS_INPUT`.
- **Release together.** v1.17 Firmware no longer sends `GPS_INPUT`, so today's SeesInterface2 gets no GPS data from it, and the new SeesInterface2 gets none from v1.13 Firmware. Release, and roll back, v1.17 Firmware with SeesInterface2 `vilas/feat/firmware_v1.17`. Don't merge the SeesInterface2 and MAVSDK branches while aircraft still fly v1.13. Merging this PR doesn't affect the fleet: `v1.17.0-dev` isn't flown until then.

**3. MAVSDK.** The branch builds against PX4 v1.17's mavlink, and decodes `SEES_GPS_RAW` into `Telemetry::GpsInput` (checked against v1.17 SITL). Still to do:
- re-pin `MAVLINK_GIT_HASH` to `33af200_dev`;
- port the other Sees MAVSDK features SeesInterface2 uses, from `michael/v3.2.0-dev`.

The details are in the MAVSDK fork's [`docs/sees/firmware_v1.17.md`](https://github.com/SEESAI/MAVSDK/blob/vilas/feat/firmware_v1.17/docs/sees/firmware_v1.17.md).

**4. SeesInterface2.**
- **Done:** the vcpkg ports (`mavsdk` 3.7.0, plus the new `mavlink-seesai` and `libevents`), and NaN handling for `heading_accuracy`. A NaN is mapped to 0, as v1.13 sent, which gets the GPS EKF's minimum yaw standard deviation.
- **To do:**
  - re-pin the `mavlink-seesai` and `mavsdk` ports;
  - migrate `SI_Plugins/SI_PluginPX4` from the MAVSDK v1.4 API to v3, mainly the `Mavsdk` constructor and the subscription handles;
  - test in SITL: GPS data, time sync, yaw, and RTCM monitoring.

  It doesn't build against MAVSDK v3.7.0 until step 3 is done.

**5. QGroundControl.** The fleet's QGroundControl (SEESAI/qgroundcontrol `v4.3.0-0.0.5`, installed by SEESAI/qgroundcontrol_launcher) doesn't know message 13000. It drops the 1 Hz stream as a CRC error, and counts each one as a lost message in its link statistics. It needs a release built with `SEES_GPS_RAW` in its MAVLink headers; no code change is needed, because the MAVLink Inspector decodes fields from the generated metadata. Hints:
- QGroundControl 4.3 uses pre-generated headers from the submodule `libs/mavlink/include/mavlink/v2.0` (`mavlink/c_library_v2`), dialect `all`.
- Generate `all` plus a verbatim copy of the `SEES_GPS_RAW` `<message>` at QGroundControl's mavlink version, and check that the generated header has length 81 and CRC_EXTRA 9. The alternative is moving QGroundControl to `33af200d`, which may need source fixes.
- The release build uses qmake: point `MAVLINKPATH`/`MAVLINK_CONF` (`QGCExternalLibs.pri`, overridable in `user_config.pri`) at the new headers.
- Check in the MAVLink Inspector against v1.17 SITL.
- Release a new AppImage, and update `QGC_APP` in the launcher's `qgroundcontrol_start.yaml`.

The new QGroundControl also works with v1.13 Firmware, so it can be released first.

**6. Hardware test and release.** Fly or bench-test v1.17 Firmware with the new SeesInterface2 and QGroundControl:
- dual-antenna `yaw` and `heading_accuracy` are finite;
- `time_utc_usec` is non-zero and SeesInterface2's time sync locks;
- `gps_id` 0 is the rover (see [GPS instance order](#gps-instance-order-gps_id));
- RTK fix and RTCM monitoring work;
- QGroundControl's MAVLink Inspector shows the message.

Then merge MAVSDK into `v3.7.0_dev` and tag it, point the SeesInterface2 `mavsdk` port at the tag, and release Firmware and SeesInterface2 together.
