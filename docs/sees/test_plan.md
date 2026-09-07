# Verification Plan: Sees Fork

* Verification plan for custom modifications applied to Sees fork of open-source [PX4 firmware](https://github.com/PX4/PX4-Autopilot).
* Branch v1.17.0-dev
* Companion to: `SEES_V1.17_MIGRATION.md`

---

## Methodology

A test case defines the behavior, how to exercise it, and pass criteria. There are four execution lanes for verification, cheapest first:

| Lane | ID | What it covers | Tooling |
|------|----|-----------|---------|
| Unit | U | Pure logic | `make tests` |
| SITL | S | Module interaction, failsafe/mode-transition logic, mavlink streams | `make px4_sitl`, MAVSDK/pymavlink scripts, `simulator_mavlink` failure injection |
| HITL (bench) | B | Driver-level behavior needing real peripherals (CAN GPS, batmon, IMU, radios) | Bench rig with the specific sensor/radio, `nsh` console, log inspection (`ulog`) |
| Flight | F | End-to-end behavior only observable in flight (GPS RTK quality, magnetic interference, RF link budget) | Test flight per existing Sees flight-test procedure |


---

## Tests

### Sees Modifications

ID | (#PR) Behaviour | Test lane: pass criteria | result (Pass/Fail)
---|-----------------|--------------------------|-------------------
B-?? | (#103) Hygrometer (SHT3x) driver enabled by default | Bench: boot board → `hygrometer status` / `dmesg` shows driver active | --
S-?? | (#107) Audible BVLOS compliant tone on kill-switch and entry into AUTO_RTL | SITL: **??** Confirm tone | -- |
B-?? | (#108) Four MAVLink instances are available | Bench: **??** | --
B-?? | (#109) SI2 receives STATUSTEXT notifications | Bench: **??** | --
F-?? | (#104) SI2 indicates whether valid RTK corrections are being received | Flight: **??** need to check flag toggling on RTK status | --
F-?? | (#101-pending) No unexpected yaw on auto-takeoff | Flight: **(**TODO: Write what exercises failure mode**)** | --

### V1.17 DEFER_UNTIL_TEST ITEMS

ID | (#PR) Behaviour | Test lane: pass criteria | result (Pass/Fail)
---|-----------------|--------------------------|-------------------


## TODO

- **Q**: Are SITL tests possible? Can SI2 interact with `px4_sitl` as a standin for cubeorange?
- Write verification statements for #102, #106 and all other DEFER_UNTIL_TEST items
