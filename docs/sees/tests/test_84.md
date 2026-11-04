**Test ID** | 84
--|--
**PR** | [84](https://github.com/SEESAI/Firmware/pull/84)
**Requirement** | When the drone tilts, PX4 does not turn SI2's forward/backward acceleration into vertical acceleration
**Test lane** | SITL

**Background**
- SI2 sends acceleration commands relative to the drone's heading. Upstream PX4 also applies the drone's tilt to them. So when the drone pitches, part of a forward or backward command becomes a downward one, and the drone sinks. The patch ignores tilt and only uses heading.
- The effect is biggest with hard stick moves. In SI2 Joystick mode, braking tilts the drone turning some horizontal velocity into vertical velocity.

**Test Method**
- Run SI2 + PX4 SITL + QGC as usual.
- In the PX4 shell, raise the stream rates (defaults are 1-1.5 Hz):
  - `mavlink stream -u 18570 -s POSITION_TARGET_LOCAL_NED -r 20`
  - `mavlink stream -u 18570 -s LOCAL_POSITION_NED -r 20`
- In QGC, open Analyze Tools > MAVLink Inspector and chart:
  - `ATTITUDE.pitch`: how much the drone tilts
  - `POSITION_TARGET_LOCAL_NED.afz`: vertical acceleration command in PX4 (positive = down)
  - `LOCAL_POSITION_NED.vz`: vertical speed (positive = down)
- Take off with SI2, switch to Joystick. Full forward stick for ~3 s, then release so SI2 brakes. Repeat 2-3 times. Screenshot the chart.
- Remove the patch, rebuild, repeat the test.

**Pass criteria**

Compare the two screenshots. Look at `afz` and `vz` each time the pitch changes.

Build | What the graph should look like
--|--
Without patch | `afz` and `vz` jump downward every time the pitch changes (speeding up and braking)
With patch | Those downward jumps are gone.

Pass if the downward jumps in the "without patch" run are clearly smaller or gone with the patch.

**Results**

Not yet run.

Build | Downward jumps when pitching? | Screenshot | Result
--|--|--|--
With patch | | |
Without patch | | |
