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

Flown 2026-11-25 on Yealm 2 with packs 104 and 105 [Notion test item](https://app.notion.com/p/seesai/PX4-1-17-vs-1-13-Flight-41df348e991880e48515e1957f72ca6f).

Check | 1.13 | 1.17 | Result
--|--|--|--
Unit tests | - | pass | Pass
Predicted % (before disarm) | 53.8 % | 54.0 % |
Observed % (after 10 min rest) | 58.2 % | 57.4 % |
Difference | +4.4 % | +3.4 % | Pass (see notes)
Under voltage message | None | None | Pass
Battery % curves look alike | | | Pass

Made with [`scripts/125_battery_soc.py`](./scripts/125_battery_soc.py).

**Notes**
- In flight, we count slightly more charge used than the rested voltage shows, which errs on the safe side.
- The flights were alike: 10.3 and 10.6 min of hover at about 60 A, using 10.2 and 10.4 Ah.
- Charge used over each whole log (10.78 and 11.03 Ah) is within 2 % of what the charger put back into the two packs (10.96 and 11.05 Ah).
- The 1.17 flight was flown in Altitude mode, not Position mode. The current was the same, so this does not affect the result.
- The last sample in each log reads up to 0.8 % lower than at 10 minutes, because of a dip when the drone was powered off. So "observed" is read at exactly 10 minutes after disarming.
