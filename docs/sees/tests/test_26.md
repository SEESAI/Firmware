**Test ID** | 26
--|--
**PR** | [26](https://github.com/SEESAI/Firmware/pull/26)
**Requirement** | v1.17 with the proposed rate controller handles like v1.13, and the new feed-forward only acts when the target attitude changes
**Test lane** | Unit + FW

**Background**
- PR #26 adds an extra push to the rate controller when the target changes sharply, for example when braking hard.
- In v1.17 we get the same behaviour with vanilla PX4, retuned gains and one small feed-forward on how fast the target attitude changes. See [26_rate_control_equivalence.md](../26_rate_control_equivalence.md).

**Test Method**

Unit test
- Run `make tests TESTFILTER=<feed-forward test name>`.
- It checks that a constant target attitude gives zero feed-forward, including on the first step.

Flight test (Whitby, same pilot, same session)
- Set the v1.17 parameters from the equivalence doc.
- Hover check on v1.17 before any hard manoeuvres.
- Fly the same set of manoeuvres on v1.13 and on v1.17, including a few hard brakes (full stick for ~3 s, then release).
- After the flight, download both logs (QGC: Analyze Tools > Log Download) and open them in Flight Review.
- Compare
	- *Roll Angular Rate* (`vehicle_rates_setpoint.roll` vs `vehicle_angular_velocity.xyz[0]`)
	- *Pitch Angular Rate* (`vehicle_rates_setpoint.pitch` vs `vehicle_angular_velocity.xyz[1]`)
	- *Actuator Outputs* on v1.13 vs v1.17

**Pass criteria**
- Unit test passes.
- Pilot reports the handling feels similar on both versions.
- In the v1.17 logs, the actual rate follows the setpoint as closely as on v1.13: no extra lag, overshoot or wobble during the brakes.
- Motor outputs on v1.17 are no closer to their limits than on v1.13.

**Results**

Not yet run.

Check | v1.13 | v1.17 | Result
--|--|--|--
Unit test | n/a | |
Handling (pilot) | | |
Rate tracking (logs) | | |
Motor outputs (logs) | | |
