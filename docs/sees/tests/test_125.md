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

Flight (Yealm, on 1.13 then 1.17, same pack, `SDLOG_MODE` = 2):
1. From a full pack, hover in Position mode for 10 minutes.
2. Land, disarm, and leave the drone powered on for 10 minutes.

Log Analysis:

3. From the log, read the battery % just before disarming (predicted) and after the rest (observed).
4. Plot battery % over time for both logs on one chart. The two curves should look alike.

**Pass criteria**
- Unit tests pass.
- On both versions, predicted and observed % agree within 3 %.
- No "Battery 1: under voltage" message in QGC during either flight.

**Results**

Check | 1.13 | 1.17 | Result
--|--|--|--
Unit tests | - | | pass
Predicted % (before disarm) | | |
Observed % (after 10 min rest) | | |
Difference | | |
Under voltage message | | |
Battery % curves look alike | | |
