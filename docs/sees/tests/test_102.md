**Test ID** | 102
--|--
**PR** | [102](https://github.com/SEESAI/Firmware/pull/102) (port of v1.13 [#84](https://github.com/SEESAI/Firmware/pull/84))
**Requirement** | When the drone tilts, PX4 does not turn SI2's forward/backward acceleration into vertical acceleration
**Test lane** | SITL

**Background**
- SI2 sends acceleration commands relative to the drone's heading. Upstream PX4 also applies the drone's tilt to them. So when the drone pitches, part of a forward or backward command becomes a downward one, and the drone sinks. The patch ignores tilt and only uses heading.
- Braking in SI2 Joystick mode tilts the drone a lot, so a large part of the horizontal command becomes vertical.

**Test Method**
- Run SI2 + PX4 SITL + QGC as usual.
- In QGC, open Analyze Tools > MAVLink Inspector and chart:
  - `ATTITUDE.pitch`: how much the drone tilts
  - `POSITION_TARGET_LOCAL_NED.afz`: vertical acceleration command in PX4 (positive = down)
  - `LOCAL_POSITION_NED.vz`: vertical speed (positive = down)
- Take off with SI2, switch to Joystick. Full forward stick for ~3 s, then release so SI2 brakes. Repeat 2-3 times. Screenshot the chart.
- Remove the patch, rebuild, repeat the test.

**Pass criteria**

Compare the two screenshots. Look at `afz` and `vz` while tilted.

Build | What the graph should look like
--|--
Without patch | `afz` and `vz` jump downward while tilted and accelerating or braking.
With patch | Those downward jumps are much smaller or gone.

Pass if the downward jumps in the "without patch" run are clearly smaller or gone with the patch.

**Results**

Not yet run.

Build | Downward jumps when pitching? | Screenshot | Result
--|--|--|--
With patch | | |
Without patch | | |
