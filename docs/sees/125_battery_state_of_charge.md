# Battery State of Charge

| | |
|---|---|
| **Status** | Since v1.13 (#30, #61, #95). Port to v1.17. |
| **Code** | `Battery::estimateStateOfCharge()` in `src/lib/battery/battery.cpp` |

## Summary

PX4 reports the battery state of charge (SoC) in `battery_status.remaining`, and Commander raises the low battery failsafes from it. On v1.13, upstream's estimate was seen to be up to 40 % off.

This patch uses the same estimator as SeesInterface2 (SI2): SoC comes from voltage on the ground and goes down by the charge used in flight.

## Why

**LiPo voltage is not linear in charge.** Upstream maps cell voltage linearly between `BAT1_V_EMPTY` and `BAT1_V_CHARGED`. Near empty, where it matters, that reads far too high (Yealm, 3.40 V to 4.35 V):

Open circuit cell voltage | 3.90 V | 3.80 V | 3.70 V | 3.60 V
--|--|--|--|--
Upstream (linear) | 53 % | 42 % | 32 % | 21 %
Chemistry curve | 57 % | 38 % | 12 % | 3 %

**Voltage under load is unreliable.** The open circuit voltage is $V_{oc} = V + R\,I$. However a LiPo's internal resistance $R$ changes with temperature, charge and age. So we need a non-voltage based state of charge measure.

## Design

Voltage tells us the charge left, but only when $I \approx 0$. Integrating current tells us the charge used, at any current, but needs a starting point.

So we use voltage on the ground and current in flight.

### 1. Voltage to SoC

$c(V)$ is the fraction of chemical charge left at open circuit voltage $V$, measured for Tattu 25 Ah HV packs (TN23.27):

$V$ | 3.300 | 3.554 | 3.678 | 3.722 | 3.757 | 3.795 | 3.838 | 3.902 | 4.002 | 4.095 | 4.203 | 4.350
--|--|--|--|--|--|--|--|--|--|--|--|--
$c$ | 0.000 | 0.026 | 0.076 | 0.175 | 0.265 | 0.377 | 0.476 | 0.578 | 0.680 | 0.779 | 0.879 | 1.000

SoC is rescaled and clamped to $[0, 1]$:

$$S(V) = \frac{c(V) - c(V_{empty})}{c(V_{charged}) - c(V_{empty})}$$

### 2. Estimator

$$
\text{SoC} \leftarrow
\begin{cases}
S(V_{oc}) & \text{disarmed} \\
\text{SoC} - \Delta Q / C & \text{armed, clamped to } [0, 1]
\end{cases}
$$

$V_{oc}$ is the cell voltage plus $R\,I$ (only when $I > 0$), filtered with $\tau$ = 1 s. $R$ is `BAT1_R_INTERNAL`, or PX4's online estimate if that is negative.

$\Delta Q$ is the charge used (found by integrating current) since the last sample, and $C$ is `BAT1_CAPACITY`. Negative $\Delta Q$ is ignored, as in v1.13 and SI2, so SoC never rises in flight.

If `BAT1_CAPACITY` is not set, SoC stays voltage-based.

### 3. Deep discharge fault

Nothing corrects SoC in flight. As an independent check, if the measured (not load-compensated) cell voltage, filtered with $\tau$ = 1 s, drops below `BAT_CRIT_V`, the battery reports PX4's standard `FAULT_DEEP_DISCHARGE`.

Upstream 1.17 Commander then:
- within `COM_SPOOLUP_TIME` of arming, disarms;
- in flight, shows "Battery 1: under voltage. Land now!" and takes no failsafe action;
- on the ground, blocks arming.

**Change from v1.13:** v1.13 only sent "Warning, Critical cell voltage X V. Land Immediately!", while armed and at most every 10 s. Blocking arming and disarming during spool-up are new. With Yealm's `BAT_CRIT_V` of 3.315 V, neither fires on a healthy pack.

## Notes for porting new PX4 Versions

This port uses `math::interpolateNXY()` for interpolation, and PX4's battery faults for reporting errors.

The curve point at 3.554 V is 0.026, SI2's value. v1.13 had 0.027 (under 0.1 % change in SoC).

SI2 treats "pack current below 20 A for 5 s" as rest, and eases toward $S(V_{oc})$ with $\tau$ = 1.6 s. Both settle to the same value at rest. In flight both subtract PX4's $\Delta Q$ over the same $C$.

Whitby's 4.2 V batteries will not be accurately represented. This is already the case in 1.13.
