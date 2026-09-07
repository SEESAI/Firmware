# Verification Plan: Sees Fork

* Verification plan for custom modifications applied to Sees fork of open-source [PX4 firmware](https://github.com/PX4/PX4-Autopilot).
* Branch v1.17.0-dev
* Companion to: `SEES_V1.17_MIGRATION.md`

---

## Methodology

A test defines the expected behavior, how to exercise it, and pass criteria. There are four execution lanes for verification, cheapest first:

| Lane | ID | What it covers | Tooling |
|------|----|-----------|---------|
| Unit | U | Pure logic | `make tests` |
| SITL | S | Module interaction, mode-transition logic, mavlink streams | `make px4_sitl`, MAVSDK/pymavlink scripts, simulation (see SeesInterface2/Notes/Simulations.md) |
| HITL (bench) | B | Driver-level behavior needing real peripherals (CAN GPS, batmon, IMU, radios) | Bench rig with the specific sensor/radio, `nsh` console, log inspection (`ulog`) |
| Flight-Whitby | FW | End-to-end behavior only observable in flight; verifiable on development drone (Whitby) | Test flight per existing Sees flight-test procedure |
| Flight-Yealm | FY | End-to-end behavior only observable in flight; verifiable only on operational drone (Yealm) | Test flight per existing Sees flight-test procedure |

---

## Tests

### SITL Tests

**Test ID** | S-001
--|--
**PR** | [101](https://github.com/SEESAI/Firmware/pull/101)
**Behaviour** | No unexpected yaw on takeoff
**Test lane** | SITL
**Result** (Pass/Fail) | --

**Test Method**
- `param set NAV_ACC_RAD 0.05` (the parameter's min, below the 0.1 guard)
- `commander takeoff`.
- **Pass** if
  - you see
  ```bash
  INFO  [navigator] Acceptance radius corrected from 0.05 to 1.0
  ```
  - `param show NAV_ACC_RAD` reads back as `1.0`
  - Log shows no yaw spin.

------
------

**Test ID** | S-002
--|--
**PR** | [109](https://github.com/SEESAI/Firmware/pull/109)
**Behaviour** | SI2 receives STATUSTEXT notifications
**Test lane** | SITL
**Result** | --

**Test Method**
- **TODO**
- Connect to the onboard instance (-m onboard, port 14540/14580) without sending a GCS heartbeat. Trigger a message (e.g. `commander arm`) and confirm STATUSTEXT arrives. This is exactly the is_onboard_controller_connected() path the fix changes.

------
------

### HITL (Bench) Tests

Test ID | B-001
--|--
**PR** | [103](https://github.com/SEESAI/Firmware/pull/103)
**Behaviour** | Hygrometer (SHT3x) driver enabled by default
**Test lane** | Bench
**Result** | --

**Test Method**
- Boot the CubeOrange with the sensor attached
- Run `sht3x status`. Verify **TODO: specify expected output**
- Run `listener sensor_hygrometer`. **TODO: specify expected output**

------
------

**Test ID** | B-002
--|--
**PR** | [107](https://github.com/SEESAI/Firmware/pull/107)
**Behaviour** | Audible BVLOS compliant tone on kill-switch and entry into AUTO_RTL
**Test lane** | Bench
**Result** | --

**Test Method**
- That the tone is triggered.
- Run `listener tune_control` and look for tune_id = TUNE_ID_BATTERY_WARNING_FAST.
- RTL: `commander takeoff`, then `commander mode auto:rtl`. The topic should repeat about every 800 ms for as long as RTL lasts (navigator_main.cpp).
- Kill: set `RC_MAP_KILL_SW` and drive that channel with `RC_CHANNELS_OVERRIDE` from pymavlink.

------
------

**Test ID** | B-003
--|--
**PR** | [108](https://github.com/SEESAI/Firmware/pull/108)
**Behaviour** | Four MAVLink instances are available
**Test lane** | Bench
**Result** | --

**Test Method**
- **TODO**
- set MAV_0..3_CONFIG to four serial ports, reboot, and confirm `mavlink status` lists 4 instances. Check free RAM as well.

------
------

**Test ID** | B-004
--|--
**PR** | [104](https://github.com/SEESAI/Firmware/pull/104)
**Behaviour** | FSI2 indicates whether valid RTK corrections are being received
**Test lane** | Bench
**Result** | --

**Test Method**
- **TODO**
- The fix only turns on the GPS_RTCM_DATA stream for onboard instances. Send fake GPS_RTCM_DATA into the GCS port (14550) and check it comes back out on the onboard port, then stops when you stop sending. That covers what this change touches.
- Whether SI2's flag tracks the real RTK status needs a real RTK GPS and base station. The simulated GPS is always a plain 3D fix. That can be done outdoors on the bench, so it doesn't need to be a flight test.

------
------



### V1.17 DEFER_UNTIL_TEST ITEMS

ID | (#PR) Behaviour | Test lane: pass criteria | result (Pass/Fail)
---|-----------------|--------------------------|-------------------


## TODO

- Write verification statements for #102, #106 and all other DEFER_UNTIL_TEST items
