**Test ID** | 33
--|--
**PR** | [33](https://github.com/SEESAI/Firmware/pull/33), [36](https://github.com/SEESAI/Firmware/pull/36), [53](https://github.com/SEESAI/Firmware/pull/53)
**Requirement** | With `SENS_MAG_LP_CUT` = 10, 50 Hz power-line noise is removed from `vehicle_magnetometer`.
**Test lane** | HITL

**Background**
- Power lines add 50 Hz noise to the magnetometer.
- The patch samples the IST8310 at ~150 Hz, filters, then averages down to 15 Hz. See [33_magnetometer_filter.md](../33_magnetometer_filter.md).

**Test Method**
- Flash v1.17 with the patch. Props off. Connect QGC over USB.
- Fix a loop of wire next to the GPS mast. Drive it from an AC power supply at 50 Hz, a few amps, current limited.
- In Analyze Tools > MAVLink Console:
	- `uorb top -1 sensor_mag`. The IST8310 is the instance at ~150 Hz. Note its rate.
- In Analyze Tools > MAVLink Inspector, chart `HIGHRES_IMU.xmag`, `ymag` and `zmag`. For each step, wait ~2 s after the `param set`, then screenshot:
	1. Supply off.
	2. Supply on, filter off: `param set SENS_MAG_LP_CUT 0`.
	3. Supply on, filter on: `param set SENS_MAG_LP_CUT 10`.
- In steps 2 and 3, run `listener vehicle_magnetometer` and note `timestamp` − `timestamp_sample`.

**Pass criteria**
- IST8310 rate is ≥ 140 Hz.
- Step 2 is visibly noisier than step 1 (a slow ~5 Hz ripple). If not, increase the current or move the loop closer.
- Step 3 looks the same as step 1.
- `timestamp` − `timestamp_sample` is ~30 ms larger in step 3 than in step 2 (filter delay correction).

**Results**

Not yet run.

Check | Screenshot | Result
--|--|--
IST8310 rate | |
Step 2 noisier than step 1 | |
Step 3 same as step 1 | |
Timestamp difference, step 3 − step 2 | |
