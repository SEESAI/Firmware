**Test ID** | 124
--|--
**PR** | [124](https://github.com/SEESAI/Firmware/pull/124) (port of v1.13 [#33](https://github.com/SEESAI/Firmware/pull/33), [#36](https://github.com/SEESAI/Firmware/pull/36), [#53](https://github.com/SEESAI/Firmware/pull/53))
**Requirement** | With `SENS_MAG_LP_CUT` = 10, 50 Hz power-line noise is removed from `vehicle_magnetometer`.
**Test lane** | SITL

**Background**
- Power lines add 50 Hz noise to the magnetometer.
- The patch samples the IST8310 at ~150 Hz, filters, then averages down to 15 Hz. See [124_magnetometer_filter.md](../124_magnetometer_filter.md).

**Test Method**
- Start PX4 SITL with an external simulator: `make px4_sitl none_iris`. It waits for the simulator to connect.
- Run [`scripts/124_mag_noise.py`](./scripts/124_mag_noise.py). It is the simulator: a drone sitting still on the ground, with 0.3 G of 50 Hz noise on the magnetometer's x axis, sampled at ~167 Hz like the IST8310.
- In the PX4 shell, send the filtered magnetometer to QGC: `mavlink stream -u 18570 -s HIGHRES_IMU -r 50`.
- In QGC, open Analyze Tools > MAVLink Inspector > `HIGHRES_IMU` and chart `xmag`.
- In the PX4 shell, `param set SENS_MAG_LP_CUT 0`, wait ~5 s and screenshot. Then `param set SENS_MAG_LP_CUT 10`, wait ~5 s and screenshot.
- After each `param set`, run `listener vehicle_magnetometer` and note how long `timestamp_sample` is before `timestamp`.

**Pass criteria**

Check | Expected
--|--
`xmag` peak to peak, filter off / on | About 26 (25–27).
`timestamp_sample` delay, filter on − off | About 32 ms, the filter delay correction.

Each first-order stage cuts the 50 Hz amplitude by $\sqrt{1 + (\frac{50}{10})^2}=\sqrt{26}$, so two stages cut it by $\approx 26$.

Each stage delays the signal by $\frac{1}{2\pi\cdot 10}\,\text{s}$, so two stages add $\approx 32\,\text{ms}$.

**Results**

Check | Value | Result
--|--|--
`xmag` peak to peak, filter off / on | 26.8 | Pass
`timestamp_sample` delay, filter on − off | 34830 − 3000 µs = 31.83 ms | Pass

Filter | `xmag` peak to peak (G)
--|--
Off | 0.072
On | 0.00268

