# Tests: Sees Fork (`v1.17.0-dev` branch)

* Verification plan for custom modifications applied to Sees fork of open-source [PX4 firmware](https://github.com/PX4/PX4-Autopilot).
* Branch v1.17.0-dev
* Companion to: `docs/sees/migration_plan_v1.17.md`

> NOTE: This document is work in progress until V1.17 migration is complete

---

## Methodology

A test defines the expected behavior, how to exercise it, and pass criteria. There are four execution lanes for verification, cheapest first:

| Lane | ID | What it covers | Tooling |
|------|----|-----------|---------|
| Unit | U | Pure logic | `make tests` |
| SITL | S | Module interaction, mode-transition logic, mavlink streams | see `SeesInterface2/Notes/Simulations.md` |
| HITL (bench) | B | Driver-level behavior needing real peripherals (CAN GPS, batmon, IMU, radios) | Bench rig with the specific sensor/radio, `nsh` console, log inspection (`ulog`) |
| Flight-Whitby | FW | End-to-end behavior only observable in flight; verifiable on development drone (Whitby) | Test flight per existing Sees flight-test procedure |
| Flight-Yealm | FY | End-to-end behavior only observable in flight; verifiable only on operational drone (Yealm) | Test flight per existing Sees flight-test procedure |

---

### Example Test Report

See [test_101.md](./test_101.md) for an example test report.

Test report has the following fields

- **Test ID**: By convention, monotonic and increasing. For simplicity, make it same as the PR number in the [Firmware repo](https://github.com/SEESAI/Firmware)
- **PR**: Direct link to associated pull request with the code change being tested
- **Requirement**: A brief statement defining the _expected_ behaviour being tested (A statement of requirement for example)
- **Test Lane**: Specify one of U, SITL, HITL, FW, FY
- **Background**: Ideally, a link to associated Jira ticket. Provides _brief_ context - what's the problem being solved and how.
- **Test method**: Bullet list of steps to be followed to verify the change
- **Pass criteria**: Specify a measurable pass/fail criteria based on the test
- **Results**: Capture the results obtained from the test

