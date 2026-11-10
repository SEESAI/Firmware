**Test ID** | 30
--|--
**PR** | [30](https://github.com/SEESAI/Firmware/pull/30), [61](https://github.com/SEESAI/Firmware/pull/61), [95](https://github.com/SEESAI/Firmware/pull/95)
**Requirement** | Disarmed, battery % follows the Reference Chemistry Curve. Armed, it falls only by the charge used. Below `BAT_CRIT_V` a deep discharge fault is reported. On Yealm it matches SI2's SoC.
**Test lane** | Unit + FY

**Background**
- Upstream reads SoC linearly from voltage, which over-estimated battery charge when they're near empty.
- This patch changes PX4 to use SI2's battery estimator. See [30_battery_state_of_charge.md](../30_battery_state_of_charge.md).
- Both SI2 and QGC should show the same battery charge, up to rounding.

**Test Method**

Unit: `make tests TESTFILTER=SeesSOC`.

Flight (Yealm, SI2 running):
1. Before arming, note the battery % in QGC and the SoC in SI2.
2. Fly a normal mission.
3. In the final hover before landing, note both again.

**Pass criteria**
- Unit tests pass.
- QGC and SI2 agree within 1 % at both readings.
- No "Battery 1: under voltage" message in QGC during the flight.

**Results**

Not yet run.

Check | QGC | SI2 | Result
--|--|--|--
Unit tests | n/a | n/a |
Before arming | | |
Final hover | | |
Under voltage message | n/a | n/a |
