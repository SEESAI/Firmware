**Test ID** | 26
--|--
**PR** | [26](https://github.com/SEESAI/Firmware/pull/26)
**Requirement** | v1.17 with the attitude feed-forward handles like v1.13 with PR #26, and the feed-forward only acts when the target attitude moves
**Test lane** | Unit + FW

**Background**
- PR #26 adds an extra push to the rate controller when the target changes sharply, for example when braking hard.
- In v1.17 we get the same behaviour from the stock rate controller, retuned gains and a feed-forward on the target's rate in the attitude loop. See [26_proposed_controller.md](../26_proposed_controller.md).

**Test Method**

Unit test
- Run `make tests TESTFILTER=SeesPID`.

Flight test (Whitby, same pilot, same session, QGC only)
- In QGC *Parameters*, set the proposed values from the [parameter table](../26_proposed_controller.md#parameter-values) and the [other changes](../26_proposed_controller.md#other-changes).
- In QGC *Analyze Tools > MAVLink Inspector*, chart `ATTITUDE` roll, pitch, rollspeed and pitchspeed, and `SERVO_OUTPUT_RAW` for the motors.
- Hover check on v1.17 before any hard manoeuvres.
- Fly the same manoeuvres on v1.13 and v1.17, including a few hard brakes (full stick for about 3 s, then release). Screenshot the charts for each brake.

**Pass criteria**
- Unit tests pass.
- Pilot reports the handling feels similar on both versions.
- After each brake, roll and pitch settle with no more overshoot or wobble than on v1.13.
- Motor outputs are no closer to their limits than on v1.13.
- Note whether rollspeed or pitchspeed reach `MC_ROLLRATE_MAX` during brakes. There the rate limit clips the push, by design.

**Results**

Not yet run.

Check | v1.13 | v1.17 | Result
--|--|--|--
Unit tests | n/a | |
Handling (pilot) | | |
Settling after brakes | | |
Motor outputs | | |
Rate limit reached | | |
