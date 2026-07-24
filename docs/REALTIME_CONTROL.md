# Real-Time Control System Documentation

## Overview

The real-time control subsystem is the orchestrator of the LEGO sorter. It manages:
- **Motor speed regulation** via PWM
- **Sensor data acquisition** (ultrasonic distance measurements)
- **Serial communication** with the Raspberry Pi (inference requests/responses)
- **Servo positioning** for bin routing
- **Event-driven timing** and state transitions

This subsystem runs on the **STM32F401RE microcontroller** and implements a straightforward event-loop architecture (not a formal finite state machine, but behaves predictably).

---

## Hardware Interface

### Motor Control (via L293D H-Bridge)

#### Connection Topology
```
STM32 PWM pin → L293D INPUT pins (1A/2A)
L293D OUTPUT pins (1Y/2Y) → DC Motor (12V side)
L293D GND, EN pins → STM32 GND, PWM speed control
```

#### PWM Configuration
- **Frequency**: ~20 kHz (typical for DC motors; avoids audible noise)
- **Duty cycle**: 0–100% (0% = stop, 100% = full speed)
- **Default**: 80% PWM on startup
- **Timer**: STM32 TIM2 or TIM3 (configurable)

#### Speed Regulation Loop
```c
// Pseudocode (actual firmware more complex)
while (true) {
    current_motor_speed_pwm = 80;  // Fixed default
    TIM_SetCompare1(TIM2, current_motor_speed_pwm * ARR / 100);
}
```

**Note**: Current implementation uses fixed PWM. Future versions could add speed feedback via encoder.

---

### Sensor Input (HC-SR04 Ultrasonic Sensors)

#### Dual Sensor Setup

| Sensor | Function | Position | Range | Pin Assignment |
|--------|----------|----------|-------|-----------------|
| HCSR04_1 (Pâlnie) | Confirms brick settled in manifold before routing | Pâlnie entry | 120–160 mm | GPIO_TRIG_1, GPIO_ECHO_1 |
| HCSR04_2 (Camera) | Triggers camera capture on brick arrival | Imaging station | 60–180 mm | GPIO_TRIG_2, GPIO_ECHO_2 |

#### Distance Measurement Protocol

**HC-SR04 Timing Sequence:**
1. STM32 sends 10 µs pulse on TRIG pin
2. Sensor measures echo time on ECHO pin
3. Distance = (echo_time_µs / 2) / 29 mm/µs
4. Result polled every ~100 ms

**Why two sensors?**
- **Sensor 2 (Camera)**: Early detection of brick at imaging station; minimal latency required
- **Sensor 1 (Pâlnie)**: Confirmation that brick has physically entered distribution manifold (not just detected by sensor 2)

#### Threshold Configuration
```c
#define CAMERA_SENSOR_MIN 60    // mm
#define CAMERA_SENSOR_MAX 180   // mm
#define PALNIE_SENSOR_MIN 120   // mm
#define PALNIE_SENSOR_MAX 160   // mm
```

**Detection logic:**
- If distance ∈ [MIN, MAX] → object present → flag set
- If distance > MAX or < MIN → no object → flag cleared
- Hysteresis: Once triggered, distance must exceed MAX + 10 mm to reset (prevents oscillation)

---

### Servo Control (50 Hz PWM)

#### Pin Configuration
- **STM32 PWM output**: TIM4_CH1 (or configurable timer)
- **Frequency**: 50 Hz (20 ms period) — standard servo frequency
- **Pulse width range**: 1 ms (0°) to 2 ms (180°)

#### Angle-to-PWM Mapping
```c
#define SERVO_PERIOD_MS 20              // 50 Hz = 20 ms period
#define SERVO_PULSE_MIN_US 1000         // 1 ms → 0°
#define SERVO_PULSE_MAX_US 2000         // 2 ms → 180°

// Servo positions for 5 bins:
int servo_angles[5] = {0, 37, 74, 111, 148};

// Calibration note: Physical servo may deviate
// Empirical adjustment required per individual servo
```

#### Positioning Sequence
1. Receive `SORT <bin>` command from Raspberry Pi
2. Look up servo angle for bin number
3. Calculate PWM duty cycle: `duty = ((servo_angles[bin] / 180) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) + SERVO_PULSE_MIN_US`
4. Set STM32 timer compare register
5. Wait for servo mechanical response (~300 ms)
6. Confirm pâlnie sensor detects brick in chute (blocking wait, up to 5 seconds)
7. Execute sort; set pâlnie repositioning delay (10 seconds)

---

## Communication Protocol (STM32 ↔ Raspberry Pi)

### Physical Layer
- **Interface**: UART serial
- **Baudrate**: 115200 bps (8 data bits, 1 stop bit, no parity)
- **Connector**: USB-TTL adapter or GPIO UART pins on Raspberry Pi

### Protocol Format: Text Lines

All messages are **newline-terminated strings** in ASCII (simple, human-readable, debuggable via serial monitor).

#### Commands: Pi → STM32

**1. Capture Image**
```
CMRA\n
```
- **Response**: None immediately; camera captures; Pi sends inference result back
- **Side effect**: STM32 triggers camera on GPIO pin; sets camera cooldown (10 seconds)
- **Error condition**: If camera cooldown active, request ignored silently

**2. Sort Brick to Bin**
```
SORT <BIN>\n
```
- **Example**: `SORT 2\n` (route to bin 2, i.e., 1×4 bricks)
- **BIN range**: 0–4 (integer, 1 digit)
- **Response**: None immediately; servo moves; pâlnie delay set (10 seconds)
- **Error condition**: Invalid bin number → ERR sent to Pi

#### Events: STM32 → Pi

**1. System Ready**
```
EVT READY\n
```
- Sent once on STM32 startup after initialization
- Pi should not send commands until this is received

**2. Brick Successfully Sorted**
```
EVT SORTED\n
```
- Sent after servo completes positioning and pâlnie sensor confirms brick in chute
- Signals Pi that system is ready for next capture request

**3. Input Queue Full**
```
EVT QUEUE_FULL\n
```
- Sent if bricks arrive faster than they can be routed
- Pi should temporarily halt sending SORT commands
- (Optional; not critical in current implementation)

**4. Classification Result (Pi → STM32)**
```
1x1 Bricks\n
```
or
```
1x2 Bricks\n
1x4 Bricks\n
2x2 Bricks\n
2x4 Bricks\n
```
- Sent by Pi after inference completes
- STM32 maps class name to bin number:
  - "1x1 Bricks" → bin 0
  - "1x2 Bricks" → bin 1
  - "1x4 Bricks" → bin 2
  - "2x2 Bricks" → bin 3
  - "2x4 Bricks" → bin 4
- Automatically triggers `SORT <BIN>` command internally

**5. Classification Error**
```
ERR\n
```
- Pi sends this if inference fails, times out, or cannot classify
- STM32 discards brick (servo does not reposition; brick passes through unchanged or falls)
- System continues to next brick

---

## Event Loop & Timing

### Main Control Loop Structure

```c
// Simplified pseudocode
void main_loop() {
    init_hardware();
    send_serial("EVT READY\n");
    
    unsigned long last_camera_trigger = 0;
    unsigned long last_sort_complete = 0;
    
    while (1) {
        // === Check sensors ===
        uint16_t camera_distance = measure_sensor_2();
        uint16_t palnie_distance = measure_sensor_1();
        
        // === Camera trigger logic ===
        if (camera_distance >= CAMERA_SENSOR_MIN && 
            camera_distance <= CAMERA_SENSOR_MAX) {
            
            if ((millis() - last_camera_trigger) > 10000) {  // 10 sec cooldown
                send_serial("CMRA\n");
                last_camera_trigger = millis();
            }
        }
        
        // === Check for Pi commands ===
        if (serial_data_available()) {
            char* cmd = read_serial_line();
            
            if (strncmp(cmd, "SORT", 4) == 0) {
                int bin = cmd[5] - '0';  // Extract bin number
                servo_set_angle(servo_angles[bin]);
                delay(300);  // Servo settling time
                
                // Wait for brick in pâlnie (blocking, max 5 sec timeout)
                unsigned long start = millis();
                while ((millis() - start) < 5000) {
                    if (palnie_distance >= PALNIE_SENSOR_MIN && 
                        palnie_distance <= PALNIE_SENSOR_MAX) {
                        send_serial("EVT SORTED\n");
                        last_sort_complete = millis();
                        break;
                    }
                }
            } 
            else if (strncmp(cmd, "ERR", 3) == 0) {
                // Pi couldn't classify; no action needed
            }
            else {
                // Unknown command
                send_serial("ERR\n");
            }
        }
        
        // === Pâlnie repositioning delay (10 sec) ===
        if ((millis() - last_sort_complete) < 10000) {
            servo_set_angle(0);  // Park servo at bin 0 position
        }
        
        delay(100);  // Main loop iteration ~100 ms
    }
}
```

### Timing Diagram

```
Time (s)  Event                           STM32 Action              Pi Action
--------  -----                           -----                     ---------
0         Boot                            EVT READY →               Waiting
3         Brick at camera                 Trigger → CMRA\n          Capture + Inference
8         Pi inference complete           ← 1x2 Bricks\n            [sends result]
8         STM32 receives result           Servo to bin 1            
8.3       Servo settled                   Wait for pâlnie sensor    
8.5       Brick enters pâlnie             EVT SORTED\n →            Ready for next
8.5-18.5  Pâlnie repositioning delay      Park servo; blocked       
18.5      Next brick detected             CMRA\n →                  Capture again
...
```

### Key Timing Constants

| Timing | Value | Purpose |
|--------|-------|---------|
| Camera cooldown | 10 s | Prevent re-trigger on same brick |
| Servo settling | 0.3 s | Allow mechanical response |
| Pâlnie confirmation timeout | 5 s | Detect jam; fall back if brick doesn't enter |
| Pâlnie repositioning delay | 10 s | Allow brick descent + chute clearing |
| Serial read timeout | 1 s | Non-blocking read; timeout prevents hang |
| Main loop iteration | ~100 ms | Sensor polling frequency |

---

## Error Handling & Recovery

### Scenario 1: Pâlnie Jam (Brick Stuck)
```
Sequence:
1. SORT <bin> command sent
2. Servo moves to position
3. STM32 polls pâlnie sensor for 5 seconds
4. Sensor never confirms brick presence (distance stays > 160 mm)
5. Timeout expires → STM32 takes no action, disables servo command
6. Brick eventually clears or operator manually removes

Result: System recovers; next brick can be sorted normally
```

### Scenario 2: Pi Inference Timeout
```
Sequence:
1. CMRA sent → camera captures
2. Pi processes image (slow inference)
3. Pi takes >5 seconds
4. STM32 still waiting for classification
5. Camera cooldown timer prevents re-capture

Result: Brick passes through unsorted; next brick starts fresh
```

### Scenario 3: Serial Line Corruption
```
Sequence:
1. Noisy serial line sends garbled: "SORx 2\n"
2. STM32 parsing fails (unrecognized command prefix)
3. STM32 sends ERR\n to Pi

Result: Ignored by Pi; system continues
```

---

## Firmware Architecture

### File Organization (Typical)
```
stm32/
├── main.c                   # Main loop, initialization
├── motor_control.c/h        # PWM setup, speed adjustment
├── sensor.c/h               # HC-SR04 polling, distance conversion
├── servo_control.c/h        # Angle mapping, PWM for servo
├── communication.c/h        # Serial TX/RX, protocol parsing
└── config.h                 # All constants (#define)
```

### Configuration Header (config.h)
```c
// Motor
#define MOTOR_PWM_FREQ_KHZ 20
#define MOTOR_DEFAULT_DUTY 80

// Sensors
#define CAMERA_SENSOR_MIN 60
#define CAMERA_SENSOR_MAX 180
#define PALNIE_SENSOR_MIN 120
#define PALNIE_SENSOR_MAX 160

// Timing (ms)
#define CAMERA_COOLDOWN_MS 10000
#define PALNIE_DELAY_MS 10000
#define PALNIE_CONFIRM_TIMEOUT_MS 5000
#define SERVO_SETTLE_MS 300

// Servo
#define SERVO_ANGLES {0, 37, 74, 111, 148}

// Serial
#define UART_BAUDRATE 115200
```

---

## Performance Characteristics

### Latency Breakdown (per brick)
| Stage | Duration | Notes |
|-------|----------|-------|
| Brick detection to CMRA send | <100 ms | Sensor polling latency |
| Pi inference | 1–3 s | Depends on YOLO11 performance |
| Classification result received | <100 ms | Serial latency |
| Servo positioning | 300 ms | Mechanical response time |
| Pâlnie confirmation wait | <500 ms | (Usually settles quickly) |
| EVT SORTED send | <100 ms | |
| **Total end-to-end** | **15–20 s** | Includes 10 s pâlnie delay |

### Throughput
- **Theoretical**: 1 brick every 10–15 seconds (without pâlnie delay)
- **Actual**: 1 brick every 15–20 seconds (includes 10 s pâlnie delay)
- **Bottleneck**: Motor speed (belt transport) and mechanical settling, not STM32 processing

---

## Testing & Debug

### Serial Terminal Testing
Use any serial monitor (PuTTY, Arduino IDE Serial Monitor, etc.) at 115200 baud:

1. **Manual brick sort**:
   ```
   SORT 0        → Routes to bin 0 (1×1)
   SORT 2        → Routes to bin 2 (1×4)
   ```

2. **Simulated classification**:
   ```
   1x2 Bricks    → STM32 automatically maps to bin 1 and sorts
   ERR           → STM32 silently discards brick
   ```

3. **Observe events**:
   ```
   EVT READY     → System booted
   EVT SORTED    → Brick dispatched
   ```

### GPIO Pin Inspection
- Oscilloscope on PWM pins: Verify frequency and duty cycle change
- Logic analyzer on UART TX/RX: Confirm protocol compliance

---

## Future Enhancements

1. **Speed feedback loop**: Add encoder → implement PID for variable speed
2. **Formal FSM**: Replace event loop with explicit state machine (UML diagram)
3. **Watchdog timer**: Auto-recover from firmware hang (WDT interrupt)
4. **Logging**: Store events to EEPROM for post-mortem analysis
5. **Protocol upgrade**: Add checksums/CRC for reliability (optional; current text protocol sufficient for short cable runs)
6. **Predictive servo**: Pre-position servo before brick arrives (requires coordinated timing with Pi)

---

## References

- **STM32F401RE Datasheet**: [Link to official datasheet]
- **L293D Motor Driver Datasheet**: [Link]
- **HC-SR04 Ultrasonic Sensor Guide**: [Common tutorial links]
- **Servo Control Theory**: PWM 50 Hz; 1–2 ms pulse = 0–180°
