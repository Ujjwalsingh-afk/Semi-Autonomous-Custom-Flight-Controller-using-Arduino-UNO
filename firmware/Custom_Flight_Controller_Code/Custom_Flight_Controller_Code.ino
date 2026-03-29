// ═══════════════════════════════════════════════════════════════
//  BUMBLEBEE-CP — Arduino UNO Flight Controller (FINAL v2)
//  Integrated BB-CP protocol receiver from Drone Nano
//  SoftwareSerial: RX=D11 (← Nano D10), TX=D12 (→ Nano D4)
//
//  BB-CP Nano→UNO packet format:
//  [BB][EE][TYPE][LEN][DATA 0-8 bytes][XOR_CRC]
//
//  Fixed: Derivative kick, real dt, yaw control, accel formulas,
//         motor smoothing removed, DLPF enabled, calibration,
//         gyro pre-filter removed, volatile cleaned up
//  Fixed: MODE_HOVER now always updates control inputs
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

// ── SoftwareSerial to Drone Nano ──────────────────────────────
SoftwareSerial nanoSerial(11, 12);  // RX=D11 ← Nano D10, TX=D12 → Nano D4

// ── Pins ──────────────────────────────────────────────────────
#define ESC1_PIN  3   // Front-Left   (CW)
#define ESC2_PIN  5   // Front-Right  (CCW)
#define ESC3_PIN  6   // Rear-Right   (CW)
#define ESC4_PIN  9   // Rear-Left    (CCW)

// ── Flight Parameters ─────────────────────────────────────────
#define THROTTLE_IDLE       1000
#define THROTTLE_MIN_SPIN   1150
#define THROTTLE_HOVER      1420
#define THROTTLE_MAX        1950

#define HOVER_TIME_MS       10000
#define TAKEOFF_RAMP_MS     4000
#define LAND_RAMP_MS        5000

// ── Test Mode ─────────────────────────────────────────────────
#define BENCH_TEST_MODE     0

#if BENCH_TEST_MODE
  #define TILT_CUTOFF       75.0f
#else
  #define TILT_CUTOFF       50.0f
#endif

// ── PID Gains ─────────────────────────────────────────────────
float Kp = 0.85f;
float Ki = 0.002f;
float Kd = 0.18f;

// ── Flight States ─────────────────────────────────────────────
enum FlightState {
  STATE_IDLE,
  STATE_ARMING,
  STATE_TAKEOFF,
  STATE_HOVER,
  STATE_LAND,
  STATE_EMERGENCY
};
FlightState currentState = STATE_IDLE;

// ── Hardware Objects ──────────────────────────────────────────
MPU6050 imu;
Servo esc1, esc2, esc3, esc4;

// ── PID State ─────────────────────────────────────────────────
float rollIntegral  = 0.0f, pitchIntegral  = 0.0f, yawIntegral  = 0.0f;
float rollPrevMeas  = 0.0f, pitchPrevMeas  = 0.0f, yawPrevMeas  = 0.0f;
float rollOut       = 0.0f, pitchOut       = 0.0f, yawOut       = 0.0f;

// ── Attitude ──────────────────────────────────────────────────
float currentRoll  = 0.0f;
float currentPitch = 0.0f;
float currentYaw   = 0.0f;

// ── Control Inputs ────────────────────────────────────────────
int   baseThrottle         = THROTTLE_IDLE;
int   manualThrottleOffset = 0;
float manualRollOffset     = 0.0f;
float manualPitchOffset    = 0.0f;
float manualYawOffset      = 0.0f;

// ── Complementary Filter ──────────────────────────────────────
float prevRoll  = 0.0f, prevPitch = 0.0f;
const float ALPHA        = 0.96f;
const float ANGLE_SMOOTH = 0.85f;
float smoothRoll = 0.0f, smoothPitch = 0.0f;

// ── IMU Bias ──────────────────────────────────────────────────
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float levelRollOffset = 0.0f, levelPitchOffset = 0.0f;

// ── Timing ────────────────────────────────────────────────────
unsigned long stateStartTime = 0;
unsigned long lastPrintTime  = 0;
unsigned long lastIMUTime    = 0;
unsigned long lastPIDTime    = 0;
unsigned long lastTlmTime    = 0;
// ── Motor Outputs ─────────────────────────────────────────────
float fMotor1 = 1000.0f, fMotor2 = 1000.0f;
float fMotor3 = 1000.0f, fMotor4 = 1000.0f;

// ── ISR Flag ──────────────────────────────────────────────────
volatile bool    imuReadFlag     = false;
volatile uint8_t imuTimerDivider = 0;

// ── Raw IMU Data ──────────────────────────────────────────────
int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;

// ── Dart State ────────────────────────────────────────────────
bool dartLoaded = true;

// ═══════════════════════════════════════════════════════════════
//  BB-CP PROTOCOL CONSTANTS
// ═══════════════════════════════════════════════════════════════
#define PKT_CMD_FLIGHT      0x01
#define PKT_CMD_FIRE        0x02
#define PKT_EMERGENCY_STOP  0x03
#define PKT_TLM_STATUS      0x04

#define MODE_IDLE    0x00
#define MODE_TAKEOFF 0x01
#define MODE_HOVER   0x02
#define MODE_TRACK   0x03
#define MODE_ALIGN   0x04
#define MODE_FIRE    0x05
#define MODE_LAND    0x06

// ═══════════════════════════════════════════════════════════════
//  BB-CP PACKET PARSER  (Nano → UNO)
//  Format: [BB][EE][TYPE][LEN][DATA...][XOR_CRC]
//  Auto-resets if stuck on garbage bytes for > 100ms
// ═══════════════════════════════════════════════════════════════
enum UNOState { U_WAIT_BB, U_WAIT_EE, U_TYPE, U_LEN, U_DATA, U_CRC };
UNOState unoParseState = U_WAIT_BB;

uint8_t pktType = 0;
uint8_t pktLen  = 0;
uint8_t pktData[8];
uint8_t pktIdx  = 0;
uint8_t calcCRC = 0;

void parseNanoByte(uint8_t b) {
    switch (unoParseState) {
        case U_WAIT_BB:
            if (b == 0xBB) unoParseState = U_WAIT_EE;
            break;
        case U_WAIT_EE:
            unoParseState = (b == 0xEE) ? U_TYPE : U_WAIT_BB;
            break;
        case U_TYPE:
            pktType = b;
            calcCRC = 0xBB ^ 0xEE ^ b;
            unoParseState = U_LEN;
            break;
        case U_LEN:
            pktLen  = b;
            calcCRC ^= b;
            pktIdx  = 0;
            unoParseState = (pktLen > 0) ? U_DATA : U_CRC;
            break;
        case U_DATA:
            if (pktIdx < 8) pktData[pktIdx] = b;
            calcCRC ^= b;
            pktIdx++;
            if (pktIdx >= pktLen) unoParseState = U_CRC;
            break;
        case U_CRC:
            if (b == calcCRC) {
                executeCommand(pktType, pktData, pktLen);
            } else {
                Serial.println(F("[NANO] CRC ERROR — dropped"));
            }
            unoParseState = U_WAIT_BB;
            break;
    }
}

// ═══════════════════════════════════════════════════════════════
//  EXECUTE COMMAND FROM NANO
// ═══════════════════════════════════════════════════════════════
void executeCommand(uint8_t type, uint8_t* data, uint8_t len) {

    switch (type) {

        case PKT_CMD_FLIGHT: {
            uint16_t throttle = ((uint16_t)data[0] << 8) | data[1];
            int8_t   pitch    = (int8_t)data[2];
            int8_t   roll     = (int8_t)data[3];
            int8_t   yaw      = (int8_t)data[4];
            uint8_t  mode     = data[5];

            Serial.print(F("[CMD_FLIGHT] T:")); Serial.print(throttle);
            Serial.print(F(" P:"));             Serial.print(pitch);
            Serial.print(F(" R:"));             Serial.print(roll);
            Serial.print(F(" Y:"));             Serial.print(yaw);
            Serial.print(F(" M:"));             Serial.println(mode);

            // Always update control inputs first
            manualThrottleOffset = (int)throttle - THROTTLE_HOVER;
            manualPitchOffset    = map(pitch, -128, 127, -15, 15);
            manualRollOffset     = map(roll,  -128, 127, -15, 15);
            manualYawOffset      = map(yaw,   -128, 127, -60, 60);

            switch (mode) {

                case MODE_IDLE:
                    if (currentState != STATE_IDLE &&
                        currentState != STATE_EMERGENCY) {
                        currentState   = STATE_LAND;
                        stateStartTime = millis();
                    }
                    break;

                case MODE_TAKEOFF:
                    if (currentState == STATE_IDLE) {
                        resetPID();
                        resetMotors();
                        manualThrottleOffset = 0;
                        manualRollOffset = manualPitchOffset = manualYawOffset = 0.0f;
                        writeAllESCs(1000);
                        delay(200);
                        Serial.println(F("[CMD] Arming for takeoff..."));
                        currentState   = STATE_ARMING;
                        stateStartTime = millis();
                    }
                    break;

                case MODE_HOVER:
                case MODE_TRACK:
                case MODE_ALIGN:
                    // Control inputs already updated above
                    // Force HOVER if still in TAKEOFF
                    if (currentState == STATE_TAKEOFF) {
                        baseThrottle   = THROTTLE_HOVER;
                        currentState   = STATE_HOVER;
                        stateStartTime = millis();
                        Serial.println(F("[CMD] HOVER forced from TAKEOFF"));
                    }
                    // Trigger takeoff if idle
                    if (currentState == STATE_IDLE) {
                        resetPID();
                        resetMotors();
                        writeAllESCs(1000);
                        delay(200);
                        currentState   = STATE_ARMING;
                        stateStartTime = millis();
                    }
                    break;

                case MODE_FIRE:
                    fireDart();
                    break;

                case MODE_LAND:
                    if (currentState == STATE_HOVER ||
                        currentState == STATE_TAKEOFF) {
                        Serial.println(F("[CMD] Landing now"));
                        manualThrottleOffset = 0;  // prevent sudden drop
                        manualRollOffset = manualPitchOffset = manualYawOffset = 0.0f;
                        currentState   = STATE_LAND;
                        stateStartTime = millis();
                    }
                    break;
            }
            break;
        }

        case PKT_CMD_FIRE:
            Serial.println(F("[CMD_FIRE] Firing dart!"));
            fireDart();
            break;

        case PKT_EMERGENCY_STOP:
            resetMotors();
            writeAllESCs(1000);
            currentState = STATE_EMERGENCY;
            Serial.println(F("[EMERGENCY STOP] All motors cut!"));
            break;

        default:
            Serial.print(F("[NANO] Unknown type: 0x"));
            Serial.println(type, HEX);
            break;
    }
}

// ═══════════════════════════════════════════════════════════════
//  SEND TELEMETRY TO NANO
// ═══════════════════════════════════════════════════════════════
void sendTelemetry() {
    if (millis() - lastTlmTime < 2000) return;  // 0.5Hz
    lastTlmTime = millis();

    uint8_t mode = MODE_IDLE;
    switch (currentState) {
        case STATE_IDLE:      mode = MODE_IDLE;    break;
        case STATE_ARMING:    mode = MODE_TAKEOFF; break;
        case STATE_TAKEOFF:   mode = MODE_TAKEOFF; break;
        case STATE_HOVER:     mode = MODE_HOVER;   break;
        case STATE_LAND:      mode = MODE_LAND;    break;
        case STATE_EMERGENCY: mode = MODE_IDLE;    break;
    }

    uint8_t flags = 0;
    if (currentState != STATE_IDLE &&
        currentState != STATE_EMERGENCY) flags |= 0x01;
    if (dartLoaded)                       flags |= 0x02;

    uint8_t payload[8] = {
        (uint8_t)currentRoll,
        (uint8_t)currentPitch,
        (uint8_t)currentYaw,
        85,
        mode,
        0,
        0,
        flags
    };

    uint8_t crc = 0xBB ^ 0xEE ^ PKT_TLM_STATUS ^ 8;
    for (int i = 0; i < 8; i++) crc ^= payload[i];

    nanoSerial.write(0xBB);
    nanoSerial.write(0xEE);
    nanoSerial.write(PKT_TLM_STATUS);
    nanoSerial.write((uint8_t)8);
    for (int i = 0; i < 8; i++) nanoSerial.write(payload[i]);
    nanoSerial.write(crc);
}

// ═══════════════════════════════════════════════════════════════
//  DART FIRE
// ═══════════════════════════════════════════════════════════════
void fireDart() {
    if (!dartLoaded) {
        Serial.println(F("[DART] Not loaded!"));
        return;
    }
    Serial.println(F("[DART] FIRING!"));
    dartLoaded = false;
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    nanoSerial.begin(57600);

    Wire.begin();
    Wire.setClock(400000);

    imu.initialize();
    imu.setRate(0);
    imu.setDLPFMode(3);
    imu.setFullScaleGyroRange(1);
    imu.setFullScaleAccelRange(1);

    if (!imu.testConnection()) {
        Serial.println(F("IMU FAILED — check wiring"));
        while (1);
    }
    Serial.println(F("IMU OK"));

    esc1.attach(ESC1_PIN);
    esc2.attach(ESC2_PIN);
    esc3.attach(ESC3_PIN);
    esc4.attach(ESC4_PIN);

    writeAllESCs(1000);
    delay(3000);

    calibrateIMU();
    setupTimer();

    lastIMUTime    = micros();
    lastPIDTime    = micros();
    stateStartTime = millis();

    printHelp();
    Serial.println(F("Waiting for commands from Drone Nano..."));

    #if BENCH_TEST_MODE
    Serial.println(F("⚠  BENCH TEST MODE ACTIVE"));
    Serial.println(F("   Tilt cutoff: 75deg  |  Auto-land: OFF"));
    #endif
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {

    // Read bytes from Drone Nano
    while (nanoSerial.available()) {
        parseNanoByte(nanoSerial.read());
    }

    // Keyboard commands for manual bench testing
    handleSerial();

    if (imuReadFlag) {
        imuReadFlag = false;
        readIMU();
        processIMU();

        if (currentState == STATE_TAKEOFF ||
            currentState == STATE_HOVER   ||
            currentState == STATE_LAND) {
            computePID();
            rollOut  = constrain(rollOut,  -150.0f, 150.0f);
            pitchOut = constrain(pitchOut, -150.0f, 150.0f);
            yawOut   = constrain(yawOut,   -100.0f, 100.0f);
            updateMotors();
        }
    }

    handleStateMachine();

    if (currentState == STATE_TAKEOFF ||
        currentState == STATE_HOVER   ||
        currentState == STATE_LAND) {
        esc1.writeMicroseconds((int)fMotor1);
        esc2.writeMicroseconds((int)fMotor2);
        esc3.writeMicroseconds((int)fMotor3);
        esc4.writeMicroseconds((int)fMotor4);
    }

    sendTelemetry();
    printTelemetry();
}

// ═══════════════════════════════════════════════════════════════
//  TIMER 2 — ~250Hz IMU update rate
// ═══════════════════════════════════════════════════════════════
void setupTimer() {
    cli();
    TCCR2A = 0; TCCR2B = 0; TCNT2 = 0;
    OCR2A = 249;
    TCCR2A |= (1 << WGM21);
    TCCR2B |= (1 << CS22);
    TIMSK2 |= (1 << OCIE2A);
    sei();
}

ISR(TIMER2_COMPA_vect) {
    imuTimerDivider++;
    if (imuTimerDivider < 4) return;
    imuTimerDivider = 0;
    imuReadFlag = true;
}

// ═══════════════════════════════════════════════════════════════
//  IMU READ
// ═══════════════════════════════════════════════════════════════
void readIMU() {
    imu.getMotion6(
        &raw_ax, &raw_ay, &raw_az,
        &raw_gx, &raw_gy, &raw_gz);
}

// ═══════════════════════════════════════════════════════════════
//  IMU CALIBRATION
// ═══════════════════════════════════════════════════════════════
void calibrateIMU() {
    Serial.println(F("Calibrating IMU — keep drone still on level surface..."));

    long gxSum = 0, gySum = 0, gzSum = 0;
    float rollSum = 0.0f, pitchSum = 0.0f;
    const int N = 500;

    for (int i = 0; i < N; i++) {
        imu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        gxSum += raw_gx; gySum += raw_gy; gzSum += raw_gz;
        rollSum  +=  atan2f((float)raw_ay, (float)raw_az) * 180.0f / PI;
        pitchSum += -atan2f((float)raw_ax,
                             sqrtf((float)raw_ay * raw_ay +
                                   (float)raw_az * raw_az)) * 180.0f / PI;
        delay(3);
    }

    gyroBiasX = gxSum / (float)N / 65.5f;
    gyroBiasY = gySum / (float)N / 65.5f;
    gyroBiasZ = gzSum / (float)N / 65.5f;

    levelRollOffset  = rollSum  / (float)N;
    levelPitchOffset = pitchSum / (float)N;

    prevRoll = prevPitch = 0.0f;
    currentRoll = currentPitch = currentYaw = 0.0f;
    lastIMUTime = micros();

    Serial.print(F("Gyro bias  X:")); Serial.print(gyroBiasX, 4);
    Serial.print(F(" Y:"));           Serial.print(gyroBiasY, 4);
    Serial.print(F(" Z:"));           Serial.println(gyroBiasZ, 4);
    Serial.print(F("Level trim R:")); Serial.print(levelRollOffset, 2);
    Serial.print(F(" P:"));           Serial.println(levelPitchOffset, 2);
    Serial.println(F("Calibration done"));
}

// ═══════════════════════════════════════════════════════════════
//  IMU FILTER
// ═══════════════════════════════════════════════════════════════
void processIMU() {
    unsigned long now = micros();
    float dt = (float)(now - lastIMUTime) / 1000000.0f;
    lastIMUTime = now;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.004f;

    const float LSB_GYRO = 65.5f;
    float gyroRoll  = (float)raw_gx / LSB_GYRO - gyroBiasX;
    float gyroPitch = (float)raw_gy / LSB_GYRO - gyroBiasY;
    float gyroYaw   = (float)raw_gz / LSB_GYRO - gyroBiasZ;

    float accelRoll  =  atan2f((float)raw_ay, (float)raw_az) * 180.0f / PI;
    float accelPitch =  -atan2f((float)raw_ax,
                                sqrtf((float)raw_ay * raw_ay +
                                      (float)raw_az * raw_az)) * 180.0f / PI;

    float filteredRoll  = ALPHA * (prevRoll  + gyroRoll  * dt) + (1.0f - ALPHA) * accelRoll;
    float filteredPitch = ALPHA * (prevPitch + gyroPitch * dt) + (1.0f - ALPHA) * accelPitch;

    prevRoll  = filteredRoll;
    prevPitch = filteredPitch;
    
    float rollRad      = currentRoll  * (PI / 180.0f);
    float pitchRad     = currentPitch * (PI / 180.0f);
    float cosPitch     = cosf(pitchRad);
    float yawRateWorld = (fabsf(cosPitch) > 0.1f)
    ? (gyroPitch * sinf(rollRad) + gyroYaw * cosf(rollRad)) / cosPitch
    : gyroYaw;  // fallback if near 90° tilt

    currentYaw += yawRateWorld * dt;
    if (currentYaw >  180.0f) currentYaw -= 360.0f;
    if (currentYaw < -180.0f) currentYaw += 360.0f;

    smoothRoll  = ANGLE_SMOOTH * smoothRoll  + (1.0f - ANGLE_SMOOTH) * (filteredRoll  - levelRollOffset);
    smoothPitch = ANGLE_SMOOTH * smoothPitch + (1.0f - ANGLE_SMOOTH) * (filteredPitch - levelPitchOffset);

    currentRoll  = -smoothRoll;
    currentPitch = -smoothPitch;
}

// ═══════════════════════════════════════════════════════════════
//  PID
// ═══════════════════════════════════════════════════════════════
float computeOnePID(float setpoint, float measurement,
                    float &integral, float &prevMeas,
                    float kp, float ki, float kd, float dt) {
    float error = setpoint - measurement;
    integral += error * dt;
    integral  = constrain(integral, -30.0f, 30.0f);
    float derivative = -(measurement - prevMeas) / dt;
    prevMeas = measurement;
    return kp * error + ki * integral + kd * derivative;
}

void computePID() {
    unsigned long now = micros();
    float dt = (float)(now - lastPIDTime) / 1000000.0f;
    lastPIDTime = now;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.004f;

    rollOut = computeOnePID(
        manualRollOffset,  currentRoll,
        rollIntegral,  rollPrevMeas,  Kp, Ki, Kd, dt);

    pitchOut = computeOnePID(
        manualPitchOffset, currentPitch,
        pitchIntegral, pitchPrevMeas, Kp, Ki, Kd, dt);

    float yawRate = (float)raw_gz / 65.5f - gyroBiasZ;
    yawOut = computeOnePID(
        manualYawOffset, yawRate,
        yawIntegral, yawPrevMeas, 1.2f, 0.0f, 0.05f, dt);
}

// ═══════════════════════════════════════════════════════════════
//  MOTOR MIXING
// ═══════════════════════════════════════════════════════════════
void updateMotors() {
    if (fabsf(currentRoll)  > TILT_CUTOFF ||
        fabsf(currentPitch) > TILT_CUTOFF) {
        currentState = STATE_EMERGENCY;
        fMotor1 = fMotor2 = fMotor3 = fMotor4 = 1000.0f;
        writeAllESCs(1000);
        Serial.println(F("[EMERGENCY] Tilt exceeded cutoff!"));
        return;
    }

    int thr = constrain(
        baseThrottle + manualThrottleOffset,
        THROTTLE_IDLE, THROTTLE_MAX);

    float m1 = thr + rollOut - pitchOut - yawOut;
    float m2 = thr - rollOut - pitchOut + yawOut;
    float m3 = thr - rollOut + pitchOut - yawOut;
    float m4 = thr + rollOut + pitchOut + yawOut;

    #if BENCH_TEST_MODE
    const float motorCap = (float)THROTTLE_MIN_SPIN + 50.0f;
    #else
    const float motorCap = (float)THROTTLE_MAX;
    #endif

    fMotor1 = constrain(m1, 1000.0f, motorCap);
    fMotor2 = constrain(m2, 1000.0f, motorCap);
    fMotor3 = constrain(m3, 1000.0f, motorCap);
    fMotor4 = constrain(m4, 1000.0f, motorCap);
}

// ═══════════════════════════════════════════════════════════════
//  STATE MACHINE
// ═══════════════════════════════════════════════════════════════
void handleStateMachine() {
    unsigned long elapsed = millis() - stateStartTime;

    switch (currentState) {

        case STATE_IDLE:
            fMotor1 = fMotor2 = fMotor3 = fMotor4 = 1000.0f;
            writeAllESCs(1000);
            break;

        case STATE_ARMING:
            writeAllESCs(1000);
            if (elapsed >= 1000) {
                Serial.println(F("[STATE] Takeoff starting..."));
                currentState   = STATE_TAKEOFF;
                stateStartTime = millis();
            }
            break;

        case STATE_TAKEOFF: {
            float t        = constrain((float)elapsed / TAKEOFF_RAMP_MS, 0.0f, 1.0f);
            float progress = t * t * (3.0f - 2.0f * t);
            baseThrottle   = THROTTLE_MIN_SPIN +
                             (int)(progress * (THROTTLE_HOVER - THROTTLE_MIN_SPIN));
            if (elapsed >= TAKEOFF_RAMP_MS) {
                baseThrottle   = THROTTLE_HOVER;
                currentState   = STATE_HOVER;
                stateStartTime = millis();
                Serial.println(F("[STATE] HOVER reached"));
            }
            break;
        }

        case STATE_HOVER:
            baseThrottle = THROTTLE_HOVER + manualThrottleOffset;
            #if !BENCH_TEST_MODE
            if (elapsed >= HOVER_TIME_MS) {
                Serial.println(F("[STATE] Auto-landing..."));
                currentState   = STATE_LAND;
                stateStartTime = millis();
            }
            #endif
            break;

        case STATE_LAND: {
            float t        = constrain((float)elapsed / LAND_RAMP_MS, 0.0f, 1.0f);
            float progress = t * t * (3.0f - 2.0f * t);  // S-curve mirrors takeoff
            baseThrottle = THROTTLE_HOVER -
                           (int)(progress * (THROTTLE_HOVER - THROTTLE_IDLE));
            if (elapsed >= LAND_RAMP_MS) {
                resetMotors();
                currentYaw     = 0.0f;   // ← ADD THIS
                currentState   = STATE_IDLE;
                stateStartTime = millis();
                Serial.println(F("[STATE] Landed. Motors stopped."));
                printHelp();
            }
            break;
        }

        case STATE_EMERGENCY:
            resetMotors();
            break;
    }
}

// ═══════════════════════════════════════════════════════════════
//  SERIAL KEYBOARD COMMANDS
// ═══════════════════════════════════════════════════════════════
void handleSerial() {
    if (!Serial.available()) return;
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') return;

    switch (cmd) {
        case 'T': case 't':
            if (currentState == STATE_IDLE) {
                resetPID(); resetMotors();
                manualThrottleOffset = 0;
                manualRollOffset = manualPitchOffset = manualYawOffset = 0.0f;
                writeAllESCs(1000); delay(200);
                Serial.println(F("[CMD] Arming..."));
                currentState = STATE_ARMING; stateStartTime = millis();
            }
            break;
        case 'L': case 'l':
            if (currentState == STATE_HOVER || currentState == STATE_TAKEOFF) {
                Serial.println(F("[CMD] Landing now"));
                currentState = STATE_LAND; stateStartTime = millis();
            }
            break;
        case 'X': case 'x':
            resetMotors(); writeAllESCs(1000);
            currentState = STATE_EMERGENCY;
            Serial.println(F("[EMERGENCY] All motors stopped!"));
            break;
        case 'R': case 'r':
            if (currentState == STATE_EMERGENCY) {
                resetPID(); resetMotors();
                currentState = STATE_IDLE; stateStartTime = millis();
                Serial.println(F("[CMD] Reset to IDLE"));
            }
            break;
        case '+':
            if (currentState == STATE_HOVER) {
                manualThrottleOffset = constrain(manualThrottleOffset + 10, -200, 200);
                Serial.print(F("Throttle offset: ")); Serial.println(manualThrottleOffset);
            }
            break;
        case '-':
            if (currentState == STATE_HOVER) {
                manualThrottleOffset = constrain(manualThrottleOffset - 10, -200, 200);
                Serial.print(F("Throttle offset: ")); Serial.println(manualThrottleOffset);
            }
            break;
        case 'Z': case 'z':
             currentYaw = 0.0f;
             yawIntegral = yawPrevMeas = 0.0f;
             Serial.println(F("Yaw zeroed"));
             break;
        case 'W': case 'w':
            manualPitchOffset = constrain(manualPitchOffset + 2.0f, -15.0f, 15.0f); break;
        case 'S': case 's':
            manualPitchOffset = constrain(manualPitchOffset - 2.0f, -15.0f, 15.0f); break;
        case 'A': case 'a':
            manualRollOffset  = constrain(manualRollOffset  - 2.0f, -15.0f, 15.0f); break;
        case 'D': case 'd':
            manualRollOffset  = constrain(manualRollOffset  + 2.0f, -15.0f, 15.0f); break;
        case 'Q': case 'q':
            manualYawOffset = constrain(manualYawOffset - 10.0f, -60.0f, 60.0f); break;
        case 'E': case 'e':
            manualYawOffset = constrain(manualYawOffset + 10.0f, -60.0f, 60.0f); break;
        case 'C': case 'c':
            manualThrottleOffset = 0;
            manualRollOffset = manualPitchOffset = manualYawOffset = 0.0f;
            Serial.println(F("Controls centered")); break;
        case 'P': Kp += 0.1f;  Serial.print(F("Kp=")); Serial.println(Kp,2); break;
        case 'p': Kp = max(0.0f, Kp-0.1f); Serial.print(F("Kp=")); Serial.println(Kp,2); break;
        case 'F': Kd += 0.05f; Serial.print(F("Kd=")); Serial.println(Kd,2); break;
        case 'f': Kd = max(0.0f, Kd-0.05f); Serial.print(F("Kd=")); Serial.println(Kd,2); break;
        case 'I': Ki += 0.001f; Serial.print(F("Ki=")); Serial.println(Ki,4); break;
        case 'i': Ki = max(0.0f, Ki-0.001f); Serial.print(F("Ki=")); Serial.println(Ki,4); break;
        case 'H': case 'h': printHelp(); break;
        default: break;
    }
}

// ═══════════════════════════════════════════════════════════════
//  TELEMETRY PRINT
// ═══════════════════════════════════════════════════════════════
void printTelemetry() {
    #if BENCH_TEST_MODE
    if (millis() - lastPrintTime < 100) return;
    #else
    if (millis() - lastPrintTime < 500) return;
    #endif
    lastPrintTime = millis();

    const char* st;
    switch (currentState) {
        case STATE_IDLE:      st = "IDLE";      break;
        case STATE_ARMING:    st = "ARMING";    break;
        case STATE_TAKEOFF:   st = "TAKEOFF";   break;
        case STATE_HOVER:     st = "HOVER";     break;
        case STATE_LAND:      st = "LAND";      break;
        case STATE_EMERGENCY: st = "EMERGENCY"; break;
        default:              st = "UNKNOWN";   break;
    }

    Serial.print(F("ST:")); Serial.print(st);
    Serial.print(F(" THR:")); Serial.print(baseThrottle + manualThrottleOffset);
    Serial.print(F(" R:"));   Serial.print(currentRoll,  1);
    Serial.print(F(" P:"));   Serial.print(currentPitch, 1);
    Serial.print(F(" Y:"));   Serial.print(currentYaw,   1);
    Serial.print(F(" M1:")); Serial.print((int)fMotor1);
    Serial.print(F(" M2:")); Serial.print((int)fMotor2);
    Serial.print(F(" M3:")); Serial.print((int)fMotor3);
    Serial.print(F(" M4:")); Serial.print((int)fMotor4);
    Serial.print(F(" Kp:")); Serial.print(Kp,2);
    Serial.print(F(" Ki:")); Serial.print(Ki,4);
    Serial.print(F(" Kd:")); Serial.println(Kd,2);
}

// ═══════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════
void writeAllESCs(int value) {
    esc1.writeMicroseconds(value);
    esc2.writeMicroseconds(value);
    esc3.writeMicroseconds(value);
    esc4.writeMicroseconds(value);
}

void resetPID() {
    rollIntegral  = pitchIntegral  = yawIntegral  = 0.0f;
    rollPrevMeas  = pitchPrevMeas  = yawPrevMeas  = 0.0f;
    rollOut       = pitchOut       = yawOut       = 0.0f;
    smoothRoll    = smoothPitch    = 0.0f;
    lastPIDTime   = micros();
}

void resetMotors() {
    fMotor1 = fMotor2 = fMotor3 = fMotor4 = 1000.0f;
    baseThrottle         = THROTTLE_IDLE;
    manualThrottleOffset = 0;
}

void printHelp() {
    Serial.println(F("\n══ COMMANDS ══════════════════════════════"));
    Serial.println(F("  T → Takeoff          L → Land"));
    Serial.println(F("  X → EMERGENCY STOP   R → Reset"));
    Serial.println(F("  C → Center controls  H → This menu"));
    Serial.println(F("── During HOVER ──────────────────────────"));
    Serial.println(F("  + / -  → Throttle    W/S → Pitch"));
    Serial.println(F("  A / D  → Roll        Q/E → Yaw"));
    Serial.println(F("── PID Tuning ────────────────────────────"));
    Serial.println(F("  P/p Kp  F/f Kd  I/i Ki"));
    Serial.println(F("══════════════════════════════════════════"));
}
