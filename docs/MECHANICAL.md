# Mechanical Subsystem Documentation

## Overview

The mechanical subsystem forms the physical backbone of the LEGO sorter. It consists of three integrated stages:
1. **Input conveyor belt** with entry mechanisms
2. **Distribution manifold (pâlnie/funnel)** with servo-controlled routing
3. **Output bins** with sorting logic

This section describes the design, components, and operational characteristics of these mechanical elements.

---

## System Overview

![Complete LEGO Sorting System](../Images/Complete%20Sorting%20Line.png)

The image above shows the fully integrated mechanical system with conveyor belts, imaging station, distribution manifold, and output bins.

---

## Stage 1: Input Conveyor Belt (Band 1)

### Purpose
Transports incoming LEGO bricks from the user input point toward the imaging station and primary detection point.

### Components

#### Motor & Drive
- **Type**: 12V DC motor
- **Control**: PWM-based speed regulation via STM32 (L293D H-bridge driver)
- **Default speed**: ~80% PWM duty cycle
- **Direction**: Continuous forward rotation

#### Conveyor Belt & Rollers
- **Material**: Rubber/polyurethane
- **Length**: [Specify from your build]
- **Width**: [Specify from your build]
- **Tension**: Manually adjustable via roller mounts

#### Brick Separation
- **Limiter components**: Pneumatic or mechanical stops positioned along Belt 1
- **Purpose**: Prevent brick-to-brick contact and ensure single-file motion
- **Adjustment**: Height/spacing tuned to handle various LEGO brick dimensions

#### Ultrasonic Sensor (Camera Trigger Sensor)
- **Sensor Type**: HC-SR04
- **Position**: Mounted above Belt 1 at imaging station (~15 cm from belt surface)
- **Detection range**: 60–180 mm (optimal: 80–150 mm)
- **Function**: Detects brick arrival at camera position; triggers image capture
- **Cooldown**: 10 seconds post-capture (prevents duplicate triggers on slow-moving bricks)
- **Electrical connection**: GPIO trigger pin, GPIO echo pin on STM32

**Why this cooldown?**
After a brick is imaged, the 10-second delay ensures the brick passes through the imaging zone before another capture is attempted. This prevents rapid re-triggering if the brick pauses or moves slowly. The delay is proportional to motor speed; faster motors = shorter delay.

---

## Stage 2: Distribution Manifold (Pâlnie)

### Purpose
Routes sorted bricks into one of 5 output bins based on classification results. Uses a servo motor to physically redirect the brick stream.

### Components

#### Servo Motor
- **Type**: Standard hobby servo (MG996R or equivalent)
- **Control signal**: PWM @ 50 Hz (1–2 ms pulse width)
- **Angles per configuration**:
  - **Coded angles in firmware**: 0°, 37°, 74°, 111°, 148°, 180°
  - **Physical reality**: Servo is uncalibrated (common with low-cost servos); actual sweep is approximately 0°, 35°, 70°, 105°, 140°, 175°
  - **Note**: Calibration drift is expected; angle values in code reflect empirical servo behavior, not theoretical mechanical angles.

#### Funnel/Chute Structure
- **Design**: 5-way plastic manifold with smooth internal surfaces
- **Brick flow**: Gravity-assisted once redirected by servo
- **Output ports**: One chute per bin (angled downward)

#### Ultrasonic Sensor (Pâlnie Confirmation Sensor)
- **Sensor Type**: HC-SR04
- **Position**: Mounted at pâlnie exit/entry point (inside or just above manifold)
- **Detection range**: 120–160 mm
- **Function**: Confirms brick has entered and settled in the manifold before routing
- **Electrical connection**: GPIO trigger pin, GPIO echo pin on STM32

**Why this sensor?**
Confirms mechanical state before servo activation. Without this confirmation, the servo might reposition while a brick is in transit, causing jams or misrouting. This sensor acts as a gate-keeper.

#### Repositioning Delay
- **Delay duration**: 10 seconds
- **Trigger**: After brick successfully routed (SORT command executed)
- **Purpose**: Allows brick to settle into bin and funnel to clear before next brick arrival

**Why 10 seconds?**
The servo + gravity system requires time for a brick to physically exit the manifold, descend the chute, and land in the correct bin. 10 seconds accounts for the worst-case descent time and servo mechanical friction. Faster motors or redesigned chutes could reduce this; this is a mechanical limitation, not a design flaw.

---

## Stage 3: Output Bins

### Structure
- **Count**: 5 physical bins (architecture supports up to 9)
- **Routing**: Each bin corresponds to one brick classification (e.g., 1×1, 1×2, 1×4, 2×2, 2×4)
- **Capacity**: Manual removal when full; optional full-level sensor (not currently implemented)

### Chute Alignment
Servo positioning ensures chutes align precisely with bin inlets:
- 0° → Bin 0 (1×1 bricks)
- 37° → Bin 1 (1×2 bricks)
- 74° → Bin 2 (1×4 bricks)
- 111° → Bin 3 (2×2 bricks)
- 148° → Bin 4 (2×4 bricks)

(Physical angles may differ; servo calibration via code adjustment)

---

## Integration Points

### Motor Control (STM32 → Motor Driver)
- **Interface**: PWM pins on STM32 (Timer-based PWM generation)
- **Driver**: L293D H-bridge (accepts PWM input, drives motor at proportional speed)
- **Frequency**: ~20 kHz PWM (standard for DC motor control)

### Sensor Input (STM32 ← Sensors)
- **Camera trigger sensor**: Edge detection on GPIO (rising edge = brick detected)
- **Pâlnie confirmation sensor**: Polling or interrupt-based range measurement
- **Both sensors**: Use STM32 UART for distance-to-time conversion (via HC-SR04 echo timing)

### Servo Control (STM32 → Servo)
- **Interface**: PWM signal (50 Hz, 1–2 ms pulse)
- **STM32 timer**: Configured for servo-compatible frequency
- **Angle mapping**: Code maintains lookup table (bin number → PWM duty cycle)

---

## Timing & State Machine

### Typical Brick Journey (Mechanical Timeline)

```
T=0s     Brick enters Belt 1
T=1-3s   Brick slides toward imaging station (speed-dependent)
T=3-5s   Brick reaches sensor 1 (camera trigger); HSENSOR1_DETECT → Pi gets CMRA command
T=5-10s  Camera captures image; Pi runs inference
T=10s    Pi sends classification (e.g., "1x2 Bricks"); STM32 receives and sets servo angle
T=10-15s Brick continues on Belt 1, enters funnel (servo already positioned)
T=15-18s Brick triggers sensor 2 (pâlnie confirmation); confirmed in correct chute
T=18s    STM32 sends SORT command to Pi; brick routed and gravity-assisted into bin
T=18-28s Pâlnie repositioning delay; servo waits, ready for next brick
T=28s    System ready for next brick; Belt 1 continues feeding
```

### Motor Speed Dependency

The entire 15–20 second per-brick throughput is **directly tied to motor speed**:
- **Current setup**: ~80% PWM → mechanical settling visible, smooth operation
- **Faster motors**: Would reduce T=1-3s and T=15-18s significantly
- **Slower motors**: Would extend delays; cooldowns might need adjustment

---

## Known Mechanical Issues & Solutions

### Issue 1: Servo Calibration Drift
- **Symptom**: Bricks occasionally land in wrong bin despite correct signal
- **Root cause**: Low-cost servo exhibits non-linear response; voltage drop affects angle precision
- **Current mitigation**: Code empirically maps angles to bins; periodic recalibration possible
- **Long-term fix**: Replace with precision servo or add closed-loop feedback

### Issue 2: Funnel Jamming (Rare)
- **Symptom**: Brick wedges in pâlnie; system blocks waiting for sensor 2
- **Root cause**: Off-axis brick entry or servo mid-rotation during brick arrival
- **Current mitigation**: Pâlnie confirmation sensor triggers timeout (~5s); Pi signals ERR, STM32 recovers
- **Prevention**: Ensure brick separation on Belt 1; smooth funnel chute surfaces

### Issue 3: Throughput Limited by Belt Speed
- **Symptom**: ~1 brick per 15–20 seconds despite faster potential
- **Root cause**: 12V DC motor torque; mechanical friction in rollers/belt
- **Current mitigation**: None (accepted as design constraint)
- **Upgrade path**: 24V motor + reinforced belt system → potential 8–10 second cycles

---

## Maintenance & Adjustment

### Routine Maintenance
- **Belt tension**: Check monthly; re-tension if slipping observed
- **Servo lubrication**: Apply light machine oil to servo gears if noise increases
- **Sensor cleaning**: Wipe HC-SR04 sensor faces with lint-free cloth (ultrasonic window)
- **Limiter inspection**: Ensure pneumatic separators hold consistent spacing

### Calibration Procedures

#### Servo Angle Calibration
1. Power on STM32 and connect serial terminal
2. Send manual servo command (e.g., `SERVO 0` for 0° position)
3. Physically measure servo horn angle with protractor
4. Adjust code constant `SERVO_ANGLE_MAP[bin]` to match physical reality
5. Repeat for all 5 bins; document offsets

#### Sensor Threshold Adjustment
1. Place a LEGO brick at nominal camera distance (~80 mm)
2. Serial monitor: Read raw distance from HC-SR04
3. Adjust `CAMERA_SENSOR_MIN` / `CAMERA_SENSOR_MAX` thresholds in firmware if needed
4. Repeat for pâlnie sensor at proper spacing (~140 mm)

---

## Design Rationale: Why These Timings Aren't Flaws

The 10-second cooldowns and settling delays often seem excessive for a ~1 brick per 15 seconds throughput. However:

1. **They're not arbitrary**: Each delay maps to a physical constraint (motor speed, servo inertia, gravity descent)
2. **They're protective**: Prevent jams, misroutes, and false triggers
3. **They scale**: Upgrade motors → reduce delays proportionally → increase throughput

The system is **not over-engineered**; it's correctly matched to its motor hardware. A faster motor automatically enables faster cycles without firmware changes (as long as mechanical tolerances hold).

---

## Future Mechanical Upgrades

1. **Higher-torque motor** (24V, higher RPM) → Belt 1 cycle time halved
2. **Precision servo** (multi-turn, feedback-equipped) → Reliable angle repeatability
3. **Smooth chute redesign** → Reduce gravity descent time (currently ~3s)
4. **Brick separator upgrade** → Accommodate mixed brick sizes simultaneously
5. **Bin-level sensor** → Auto-stop when bin full (currently manual check)

---
