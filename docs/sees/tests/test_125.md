**Test ID** | 125
--|--
**PR** | [125](https://github.com/SEESAI/Firmware/pull/125) (port of v1.13 [#30](https://github.com/SEESAI/Firmware/pull/30), [#61](https://github.com/SEESAI/Firmware/pull/61), [#95](https://github.com/SEESAI/Firmware/pull/95))
**Requirement** | The battery % PX4 predicts by counting charge in flight equals the % it observes from the rested pack voltage after landing.
**Test lane** | Unit + FY

**Background**
- Upstream reads battery % linearly from voltage, which over-estimates the charge left when the battery is near empty.
- The patch reads battery % from the Reference Chemistry Curve while disarmed, and subtracts the charge used while armed. See [125_battery_state_of_charge.md](../125_battery_state_of_charge.md).
- After landing, the rested voltage shows the true charge left. If the in-flight % was right, the two agree.

**Test Method**

Unit: `make tests TESTFILTER=SeesSOC`.

Flight (Yealm):
1. Fly a normal mission.
2. Just before disarming, note the battery % in QGC (predicted).
3. Leave the drone powered and disarmed. After a few minutes the battery voltage in QGC should have stopped changing, note the battery % (observed).

**Pass criteria**
- Unit tests pass.
- Predicted and observed % agree within 3 %.
- No "Battery 1: under voltage" message in QGC during the flight.

**Results**

Not yet run.

Check | Value | Result
--|--|--
Unit tests |  | pass
Predicted % (before disarm) | |
Observed % (rested) | |
Difference | |
Under voltage message | |
