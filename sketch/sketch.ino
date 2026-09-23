/*
  UNO Q real-time motion and safety firmware
  ===========================================

  Responsibilities kept on the STM32U585:
  - Initialise the PCA9685 at 50 Hz.
  - Keep all six logical servos at a known 90-degree home value on startup.
  - Enforce servo angle and pulse-width limits.
  - Interpolate all six axes with a smoothstep profile.
  - Monitor a fail-safe, normally-closed emergency-stop input.
  - Disable PCA9685 outputs through its OE pin.
  - Stop on Linux heartbeat loss.
  - Expose short, non-blocking RPC functions to Linux.

  Wiring:
  - PCA9685 SDA -> UNO Q D20
  - PCA9685 SCL -> UNO Q D21
  - PCA9685 OE  -> UNO Q D3
  - NC E-stop auxiliary contact between UNO Q D2 and GND
  - PCA9685 logic VCC -> 3.3 V
  - PCA9685 V+ -> external regulated 6 V
  - All grounds common

  A real emergency-stop must also interrupt the external servo supply through
  appropriately rated hardware. This firmware is not a safety-rated controller.
*/

#include <Arduino_RouterBridge.h>
#include <Wire.h>

// ---------------------------- Hardware configuration ----------------------------

static constexpr uint8_t PCA9685_ADDRESS = 0x40;
static constexpr uint8_t ESTOP_PIN = D2;
static constexpr uint8_t PCA_OE_PIN = D3;  // Active LOW
static constexpr uint8_t SERVO_COUNT = 6;
static constexpr float PWM_FREQUENCY_HZ = 50.0f;
static constexpr uint32_t MOTION_UPDATE_MS = 20;

// PCA9685 registers.
static constexpr uint8_t REG_MODE1 = 0x00;
static constexpr uint8_t REG_MODE2 = 0x01;
static constexpr uint8_t REG_LED0_ON_L = 0x06;
static constexpr uint8_t REG_PRESCALE = 0xFE;

// Fault values are mirrored by python/fruit_sorter/robot_client.py.
enum FaultCode : int {
  FAULT_NONE = 0,
  FAULT_PHYSICAL_ESTOP = 1,
  FAULT_WATCHDOG = 2,
  FAULT_SOFTWARE_ESTOP = 3,
  FAULT_INVALID_COMMAND = 4,
  FAULT_I2C_FAILURE = 5
};

struct ServoCalibration {
  uint8_t channel;
  float minDeg;
  float maxDeg;
  uint16_t minPulseUs;
  uint16_t maxPulseUs;
};

ServoCalibration servoConfig[SERVO_COUNT] = {
  {0, 0.0f, 180.0f, 500, 2500},
  {1, 0.0f, 180.0f, 500, 2500},
  {2, 0.0f, 180.0f, 500, 2500},
  {3, 0.0f, 180.0f, 500, 2500},
  {4, 0.0f, 180.0f, 500, 2500},
  {5, 0.0f, 180.0f, 500, 2500}
};

float currentAngle[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};
float startAngle[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};
float targetAngle[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};

volatile int faultCode = FAULT_NONE;
volatile bool outputsEnabled = false;
volatile bool motionBusy = false;
volatile uint32_t lastHeartbeatMs = 0;
uint32_t watchdogTimeoutMs = 2000;

uint32_t motionStartMs = 0;
uint32_t motionDurationMs = 0;
uint32_t lastMotionUpdateMs = 0;

// ---------------------------- PCA9685 low-level driver ---------------------------

bool pcaWrite8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool pcaWritePwm(uint8_t channel, uint16_t onTick, uint16_t offTick) {
  if (channel > 15) {
    return false;
  }

  const uint8_t reg = REG_LED0_ON_L + 4 * channel;
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);
  Wire.write(onTick & 0xFF);
  Wire.write((onTick >> 8) & 0x0F);
  Wire.write(offTick & 0xFF);
  Wire.write((offTick >> 8) & 0x0F);
  return Wire.endTransmission() == 0;
}

bool pcaBegin() {
  // Totem-pole outputs and output change on STOP.
  if (!pcaWrite8(REG_MODE2, 0x04)) {
    return false;
  }

  // Sleep before changing PRE_SCALE.
  if (!pcaWrite8(REG_MODE1, 0x10)) {
    return false;
  }

  const float prescaleValue =
      25000000.0f / (4096.0f * PWM_FREQUENCY_HZ) - 1.0f;
  const uint8_t prescale = static_cast<uint8_t>(prescaleValue + 0.5f);

  if (!pcaWrite8(REG_PRESCALE, prescale)) {
    return false;
  }

  // Auto-increment + ALLCALL, then restart.
  if (!pcaWrite8(REG_MODE1, 0x21)) {
    return false;
  }
  delay(5);
  return pcaWrite8(REG_MODE1, 0xA1);
}

bool writeServoAngle(uint8_t logicalIndex, float angleDeg) {
  if (logicalIndex >= SERVO_COUNT) {
    return false;
  }

  const ServoCalibration &cfg = servoConfig[logicalIndex];
  if (angleDeg < cfg.minDeg || angleDeg > cfg.maxDeg) {
    return false;
  }

  const float normalised =
      (angleDeg - cfg.minDeg) / (cfg.maxDeg - cfg.minDeg);
  const float pulseUs =
      cfg.minPulseUs + normalised * (cfg.maxPulseUs - cfg.minPulseUs);
  const uint16_t ticks = static_cast<uint16_t>(
      pulseUs * PWM_FREQUENCY_HZ * 4096.0f / 1000000.0f + 0.5f);

  return pcaWritePwm(cfg.channel, 0, ticks);
}

bool writeAllCurrentAngles() {
  for (uint8_t index = 0; index < SERVO_COUNT; ++index) {
    if (!writeServoAngle(index, currentAngle[index])) {
      return false;
    }
  }
  return true;
}

// ------------------------------- Safety functions --------------------------------

bool physicalEstopAsserted() {
  /*
    Fail-safe NC wiring:
    - Normal contact closed to GND -> LOW.
    - Pressed/open/broken wire -> INPUT_PULLUP reads HIGH.
  */
  return digitalRead(ESTOP_PIN) == HIGH;
}

void disableOutputsHardware() {
  digitalWrite(PCA_OE_PIN, HIGH);
  outputsEnabled = false;
  motionBusy = false;
}

void latchFault(int code) {
  faultCode = code;
  disableOutputsHardware();
}

bool heartbeatFresh() {
  return (millis() - lastHeartbeatMs) <= watchdogTimeoutMs;
}

// ---------------------------- Non-blocking motion engine --------------------------

float smoothstep(float t) {
  // Zero velocity at both ends, reducing mechanical jerk.
  return t * t * (3.0f - 2.0f * t);
}

void updateMotion() {
  if (!motionBusy || !outputsEnabled || faultCode != FAULT_NONE) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastMotionUpdateMs < MOTION_UPDATE_MS) {
    return;
  }
  lastMotionUpdateMs = now;

  const uint32_t elapsed = now - motionStartMs;
  float t = motionDurationMs == 0
      ? 1.0f
      : static_cast<float>(elapsed) / static_cast<float>(motionDurationMs);
  if (t > 1.0f) {
    t = 1.0f;
  }

  const float blend = smoothstep(t);
  for (uint8_t index = 0; index < SERVO_COUNT; ++index) {
    currentAngle[index] =
        startAngle[index] + (targetAngle[index] - startAngle[index]) * blend;
    if (!writeServoAngle(index, currentAngle[index])) {
      latchFault(FAULT_I2C_FAILURE);
      return;
    }
  }

  if (t >= 1.0f) {
    for (uint8_t index = 0; index < SERVO_COUNT; ++index) {
      currentAngle[index] = targetAngle[index];
    }
    motionBusy = false;
  }
}

// -------------------------------- RPC callbacks -----------------------------------

int rpcPing() {
  return 100;  // Firmware version 1.00
}

bool rpcHeartbeat() {
  lastHeartbeatMs = millis();
  return true;
}

int rpcGetFault() {
  return faultCode;
}

bool rpcMotionBusy() {
  return motionBusy;
}

bool rpcOutputsEnabled() {
  return outputsEnabled;
}

bool rpcPhysicalEstop() {
  return physicalEstopAsserted();
}

int rpcConfigureWatchdog(int timeoutMs) {
  if (timeoutMs < 500 || timeoutMs > 30000) {
    return FAULT_INVALID_COMMAND;
  }
  watchdogTimeoutMs = static_cast<uint32_t>(timeoutMs);
  return FAULT_NONE;
}

int rpcConfigureServo(
    int logicalIndex,
    int channel,
    float minDeg,
    float maxDeg,
    int minPulseUs,
    int maxPulseUs) {
  if (logicalIndex < 0 || logicalIndex >= SERVO_COUNT ||
      channel < 0 || channel > 15 ||
      minDeg < 0.0f || maxDeg > 180.0f || maxDeg <= minDeg ||
      minPulseUs < 300 || maxPulseUs > 3000 ||
      maxPulseUs <= minPulseUs) {
    return FAULT_INVALID_COMMAND;
  }

  ServoCalibration &cfg = servoConfig[logicalIndex];
  cfg.channel = static_cast<uint8_t>(channel);
  cfg.minDeg = minDeg;
  cfg.maxDeg = maxDeg;
  cfg.minPulseUs = static_cast<uint16_t>(minPulseUs);
  cfg.maxPulseUs = static_cast<uint16_t>(maxPulseUs);

  if (currentAngle[logicalIndex] < minDeg ||
      currentAngle[logicalIndex] > maxDeg) {
    return FAULT_INVALID_COMMAND;
  }
  return FAULT_NONE;
}

int rpcResetEstop() {
  if (physicalEstopAsserted()) {
    return FAULT_PHYSICAL_ESTOP;
  }
  if (!heartbeatFresh()) {
    return FAULT_WATCHDOG;
  }

  faultCode = FAULT_NONE;
  motionBusy = false;
  return FAULT_NONE;
}

int rpcEnableOutputs() {
  if (physicalEstopAsserted()) {
    latchFault(FAULT_PHYSICAL_ESTOP);
    return FAULT_PHYSICAL_ESTOP;
  }
  if (!heartbeatFresh()) {
    latchFault(FAULT_WATCHDOG);
    return FAULT_WATCHDOG;
  }
  if (faultCode != FAULT_NONE) {
    return faultCode;
  }

  // Load valid pulse values before lowering active-low OE.
  if (!writeAllCurrentAngles()) {
    latchFault(FAULT_I2C_FAILURE);
    return FAULT_I2C_FAILURE;
  }

  digitalWrite(PCA_OE_PIN, LOW);
  outputsEnabled = true;
  return FAULT_NONE;
}

int rpcDisableOutputs() {
  disableOutputsHardware();
  return FAULT_NONE;
}

int rpcSoftwareEstop() {
  latchFault(FAULT_SOFTWARE_ESTOP);
  return FAULT_SOFTWARE_ESTOP;
}

int rpcStartMotion(
    float a0,
    float a1,
    float a2,
    float a3,
    float a4,
    float a5,
    int durationMs) {
  if (faultCode != FAULT_NONE) {
    return faultCode;
  }
  if (!outputsEnabled) {
    return FAULT_INVALID_COMMAND;
  }
  if (!heartbeatFresh()) {
    latchFault(FAULT_WATCHDOG);
    return FAULT_WATCHDOG;
  }
  if (durationMs < 50 || durationMs > 10000) {
    return FAULT_INVALID_COMMAND;
  }

  const float requested[SERVO_COUNT] = {a0, a1, a2, a3, a4, a5};
  for (uint8_t index = 0; index < SERVO_COUNT; ++index) {
    const ServoCalibration &cfg = servoConfig[index];
    if (requested[index] < cfg.minDeg || requested[index] > cfg.maxDeg) {
      return FAULT_INVALID_COMMAND;
    }
  }

  for (uint8_t index = 0; index < SERVO_COUNT; ++index) {
    startAngle[index] = currentAngle[index];
    targetAngle[index] = requested[index];
  }

  motionDurationMs = static_cast<uint32_t>(durationMs);
  motionStartMs = millis();
  lastMotionUpdateMs = 0;
  motionBusy = true;
  return FAULT_NONE;
}

// ------------------------------------ Setup ---------------------------------------

void setup() {
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(PCA_OE_PIN, OUTPUT);

  // Outputs remain disabled until Linux configures limits, sends a heartbeat,
  // clears the safety state, and explicitly enables them.
  disableOutputsHardware();

  Wire.begin();
  Wire.setClock(400000);

  if (!pcaBegin()) {
    faultCode = FAULT_I2C_FAILURE;
  } else {
    // Preload the required 90-degree startup/home pulse values while OE is HIGH.
    if (!writeAllCurrentAngles()) {
      faultCode = FAULT_I2C_FAILURE;
    }
  }

  lastHeartbeatMs = millis();

  Bridge.begin();

  // Hardware-changing callbacks use provide_safe so they execute in loop context.
  Bridge.provide_safe("configure_watchdog", rpcConfigureWatchdog);
  Bridge.provide_safe("configure_servo", rpcConfigureServo);
  Bridge.provide_safe("reset_estop", rpcResetEstop);
  Bridge.provide_safe("enable_outputs", rpcEnableOutputs);
  Bridge.provide_safe("disable_outputs", rpcDisableOutputs);
  Bridge.provide_safe("software_estop", rpcSoftwareEstop);
  Bridge.provide_safe("start_motion", rpcStartMotion);

  // These callbacks only read or atomically update small volatile values.
  Bridge.provide("ping", rpcPing);
  Bridge.provide("heartbeat", rpcHeartbeat);
  Bridge.provide("get_fault", rpcGetFault);
  Bridge.provide("motion_busy", rpcMotionBusy);
  Bridge.provide("outputs_enabled", rpcOutputsEnabled);
  Bridge.provide("physical_estop", rpcPhysicalEstop);

  if (physicalEstopAsserted()) {
    latchFault(FAULT_PHYSICAL_ESTOP);
  }
}

void loop() {
  // Physical E-stop is checked on every pass and has priority over all motion.
  if (physicalEstopAsserted() && faultCode != FAULT_PHYSICAL_ESTOP) {
    latchFault(FAULT_PHYSICAL_ESTOP);
  }

  if (outputsEnabled && !heartbeatFresh()) {
    latchFault(FAULT_WATCHDOG);
  }

  updateMotion();
  delay(1);
}
