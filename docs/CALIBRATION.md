# Elevator calibration and limits

Everything the elevator needs before it is allowed to move, and the reasoning behind each rule.
The README covers day-to-day operation; this file is the part that explains *why the numbers are
what they are*, including one mistake this project shipped with.

## 1. The one number, and the rig it was wrong for

The elevator's only position protection is computed in Java from TalonFX rotor readings. That makes
the calibration value a safety input, so it is worth being precise about what it means:

> **travel** = the number of rotor rotations a given motor covers between the calibrated zero
> (elevator fully down) and full extension, in the extension-positive convention.

This project originally asked for **one** number: the *average* of the two motors' readings at the
top, divided by both. That is correct only when both motors cover the same distance — and on the
bench elevator it was ported from, they do not. ArmDemo measured **3.4** rotations on motor 1 and
**14.5** on motor 2 for the same physical stroke, so it kept a limit per motor
(`clone-projects/ArmDemo/src/main/java/frc/robot/subsystems/ElevatorSubsystem.java:322-328`).

Work through what one averaged number does on that rig. The average is `(3.4 + 14.5) / 2 = 8.95`,
and the limit rule is "whichever motor reaches its share of the travel first stops both":

| Stage height | motor 1 / 8.95 | motor 2 / 8.95 | what the shared model concludes |
| --- | ---: | ---: | --- |
| 62% extended | 0.24 | **1.00** (motor 2 at 8.95 of 14.5) | "at the upper limit" — stop |
| 100% extended | 0.38 | 1.62 | unreachable |

So the elevator stops at **62% of its own stroke**, and the half-travel preset lands at **31%**,
with nothing in the log to say so. It fails in the safe direction (short, never past the top),
which is exactly why it went unnoticed.

Per-motor normalization fixes it: each motor's fraction reaches 1.0 at the same physical height, so
"whichever arrives first" means what it is supposed to mean. Filling **both** travels is now
supported (`MOTOR_1_MAX_EXTENSION_ROTATIONS`, `MOTOR_2_MAX_EXTENSION_ROTATIONS`), and the single
shared value remains available for a genuinely symmetric rig.

### The diagnostic, because the constants cannot catch this

If only a shared value is filled in, the configuration itself looks self-consistent — both travels
are the same number, so no arithmetic on the constants can reveal the mismatch. Only the encoders
know. So `periodic()` compares the two live readings once either motor is past 20% of travel:

```
MeasuredTravelRatio = max(|motor1|, |motor2|) / min(|motor1|, |motor2|)
CalibrationSuspect  = shared value in use AND MeasuredTravelRatio > TRAVEL_ASYMMETRY_WARNING_RATIO
```

On a 1:1 rig that ratio sits at 1.0. On the geared rig above it reads ~4.3 and
`Elevator/CalibrationSuspect` latches true — during the same hand-raise measurement that produces
the number, so the fix and the symptom show up in the same log file.

## 2. Two signs, not one

Whether the encoder reading rises or falls while the elevator extends, and which voltage polarity
extends it, are two independent physical facts: one is how the encoder is mounted, the other is how
the motor is wired. They used to be folded into a single signed travel constant, which meant one
number had to encode both and the code could not tell them apart. They are now separate:

| Constant | Meaning | Default (this bench rig) |
| --- | --- | --- |
| `EXTENSION_ROTOR_SIGN` | sign of the raw rotor reading while extending | `-1` — the reading falls |
| `EXTENSION_VOLTAGE_SIGN` | sign of the motor-1 voltage that extends | `+1` |

Calibration travels are therefore **positive magnitudes**. A negative travel is no longer a style of
calibration; it counts as unfilled input and the elevator refuses to move.

**If your rig is wound the other way**, flip `EXTENSION_ROTOR_SIGN`. If the mechanism then runs away
from its limits instead of into them, flip `EXTENSION_VOLTAGE_SIGN` as well. Both are one-line
changes that require a rebuild — deliberately, since they are wiring facts rather than tuning.

### The fault that catches a wrong sign

Getting either sign backwards is not a cosmetic error: it makes the mechanism drive *away* from the
limit it was supposed to stop at, which is the one failure mode a software-limited elevator must not
have. So the subsystem watches for it. While a movement voltage above 0.5 V has been applied for at
least 0.15 s, the change in travel fraction must agree with the direction commanded. Contradirectional
travel over 2% of the stroke latches `Elevator/DirectionReversed`, which cuts the output and refuses
every further movement until the constants are fixed.

A mechanism that draws voltage and does **not** move at all is reported as
`Elevator/MotionDetected = false` rather than latched as a fault: a engaged brake, a bound stage, or
a stalled motor should be visible in the log without the code declaring the whole elevator unsafe.

## 3. What the limits do and do not protect against

This is the honest boundary of the design, and the part to read before trusting it on a mechanism
with real mass behind it.

**In place:**

- Limit checks run inside `periodic()`, immediately after that cycle's sensor poll, so a limit that
  triggers cuts the output in the cycle it is seen. (Commands used to test limits in `execute()`,
  which `CommandScheduler` runs *before* the subsystem's `periodic()` — every decision was made on
  a 20 ms-old position and the last approved voltage stayed on the wires for a cycle past the stop.)
- Positions are latency-compensated against their own velocity signal
  (`BaseStatusSignal.getLatencyCompensatedValueAsDouble`), which removes the drive-to-robot read lag.
- A drive that stops answering, a failed status transaction, or a reading older than
  `Elevator Sensor Timeout (ms)` makes the sensors untrusted, and untrusted sensors mean zero volts.
  A frozen position that still reads "not at the limit" is the dangerous case, and it is treated as
  untrustworthy rather than as data. The age test alone can be turned off with `0` (see §4); the
  "never answered" and "transaction failed" tests cannot.
- The acceleration ramp acts on the signed motor-1 voltage, so a reversal passes through 0 V
  instead of snapping to the opposite polarity, and the speed ceiling cuts the drive voltage when
  the faster motor is already at its travel cap.

**Not in place, by choice:**

- No closed-loop position tracking on the limit-style moves: they are open-loop voltage, exactly as
  validated on the bench. The preset controller is proportional only.
- No hardware limit switch and no homing routine. Zero is where you last said it was, which is why
  the power-on rule in the README exists.
- No drive-layer soft limits yet. Phoenix 6 does expose them — `TalonFXConfigurator.apply(new
  SoftwareLimitSwitchConfigs().withForwardSoftLimitThreshold(...).withForwardSoftLimitEnable(true))`
  is confirmed present in the 26.3.0 jar — but three things about it could not be settled from here:
  which sensor the thresholds are measured against on this firmware, whether they clamp a
  `VoltageOut`-style request or only closed-loop ones, and whether they need a Phoenix Pro license
  key to take effect. A second safety layer that silently does nothing is worse than none, because
  it reads like protection. So it is listed as the next on-robot task instead of being wired up on
  assumptions. Verify all three on the bench before adding it; if it holds, it becomes genuinely
  independent of the code loop, which nothing here currently is.

## 4. Reading a log

`Elevator/*`, in the order worth scanning:

1. `Calibrated`, `SensorsTrusted`, `DirectionReversed`, `CalibrationSuspect` — the four flags that
   explain any refusal to move.
2. `MaxExtensionRotations` / `Motor2MaxExtensionRotations` / `CalibrationSource` — what the code
   thinks the stroke is, and whether it came from one number or two.
3. `ExtensionFraction`, `Motor1TravelFractionPerSecond`, `Motor2TravelFractionPerSecond` — where it
   is and how fast it is going, in units that are comparable between rigs.
4. `Motor1PositionAgeMilliseconds` — how stale the readings are getting. `-1` means that drive never
   reported at all. If the elevator refuses to move and these are large, either the bus is unhappy
   or the age budget needs raising; setting the timeout to `0` isolates which.
5. `Mode`, `TargetExtensionFraction`, `PositionErrorFraction`, `PositionArrived` — what was asked
   for and how close it got.
6. `LastMoveActionName`, `LastMoveDurationSeconds` — per-action timing for cycle estimates.

## 5. 中文要点

- **为什么不再是"填一个平均值"**：两台电机行程不同时（ArmDemo 实测 3.4 / 14.5 圈），用同一个平均
  值做分母会让电梯在 **62%** 行程处停住，中位预设落在 **31%**，而且日志里看不出异常。现在按各电机
  自身行程归一化，两路分数在同一物理高度同时到达 1.0。
- **对称机构仍然只填一个数**：`MAX_EXTENSION_ROTATIONS` 照旧可用；不对称的机构填两个分电机值。
  如果只填了共享值而两路读数实际差很多，`Elevator/CalibrationSuspect` 会在你举到顶做标定的同一次
  日志里亮起。
- **两个符号分开**：编码器绕向 `EXTENSION_ROTOR_SIGN` 与电压方向 `EXTENSION_VOLTAGE_SIGN` 是两件
  独立的事，标定值因此一律是正数。填反了任一符号，机构会朝限位的反方向跑，所以运行时会锁存
  `DirectionReversed` 故障并停止一切运动——不要靠"看起来能动"来判断符号对不对。
- **传感器不可信就不动**：掉线、事务失败、读数过期，任何一种都让输出归零；年龄阈值可以在
  Dashboard 上设 0 单独关掉，其余检查不可关闭。
- **驱动器层软限位暂未启用**：API 已核实存在，但作用传感器、是否约束电压请求、是否需要 Phoenix Pro
  授权三点无法在台架外确认；静默失效的第二层保护比没有保护更危险，留作上机待办（见 §3）。
