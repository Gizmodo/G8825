#include "GBasic.h"

/*
 * Microstepping resolution truth table (Page 13 of DRV8825 pdf)
 * 0bMODE2,MODE1,MODE0 for 1,2,4,8,16,32 microsteps
 */
const uint8_t GBasic::MS_TABLE[] = {0b000, 0b001, 0b010, 0b011, 0b100, 0b111};

/*
 * Basic connection: only DIR, STEP are connected.
 * Microstepping controls should be hardwired.
 */
GBasic::GBasic(short steps, short dir_pin, short step_pin, Shiftduino &param,
               int index)
    : GBasic(steps, dir_pin, step_pin, PIN_UNCONNECTED, param, index) {}

GBasic::GBasic(short steps, short dir_pin, short step_pin, short enable_pin,
               Shiftduino &param, int index)
    : motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin),
      enable_pin(enable_pin), _sd(param), _index(index) {
  steps_to_cruise = 0;
  steps_remaining = 0;
  dir_state = 0;
  steps_to_brake = 0;
  step_pulse = 0;
  cruise_step_pulse = 0;
  rest = 0;
  step_count = 0;
}
GBasic::GBasic(short steps, short dir_pin, short step_pin, short mode0_pin,
               short mode1_pin, short mode2_pin, Shiftduino &param, int index)
    : GBasic(steps, dir_pin, step_pin, PIN_UNCONNECTED, mode0_pin, mode1_pin,
             mode2_pin, param, index) {}

GBasic::GBasic(short steps, short dir_pin, short step_pin, short enable_pin,
               short mode0_pin, short mode1_pin, short mode2_pin,
               Shiftduino &param, int index)
    : motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin),
      enable_pin(enable_pin), mode0_pin(mode0_pin), mode1_pin(mode1_pin),
      mode2_pin(mode2_pin), _sd(param), _index(index) {
  steps_to_cruise = 0;
  steps_remaining = 0;
  dir_state = 0;
  steps_to_brake = 0;
  step_pulse = 0;
  cruise_step_pulse = 0;
  rest = 0;
  step_count = 0;
}

/*
 * Initialize pins, calculate timings etc
 */
void GBasic::beginChild(float rpm, short microsteps) {
  pinMode(dir_pin, OUTPUT);
  _sd.setPin(dir_pin, HIGH);

  pinMode(step_pin, OUTPUT);
  _sd.setPin(step_pin, LOW);

  if IS_CONNECTED (enable_pin) {
    pinMode(enable_pin, OUTPUT);
    disable();
  }

  this->rpm = rpm;
  setMicrostep(microsteps);

  enable();
}
void GBasic::begin(float rpm, short microsteps) {
  GBasic::beginChild(rpm, microsteps);
  if (!IS_CONNECTED(mode0_pin) || !IS_CONNECTED(mode1_pin) ||
      !IS_CONNECTED(mode2_pin)) {
    return;
  }

  pinMode(mode0_pin, OUTPUT);
  pinMode(mode1_pin, OUTPUT);
  pinMode(mode2_pin, OUTPUT);
}

/*
 * Set target motor RPM (1-200 is a reasonable range)
 */
void GBasic::setRPM(float rpm) {
  if (this->rpm == 0) { // begin() has not been called (old 1.0 code)
    begin(rpm, microsteps);
  }
  this->rpm = rpm;
}

/*
 * Set stepping mode (1:microsteps)
 * Allowed ranges for GBasic are 1:1 to 1:128
 */
short GBasic::setMicrostepChild(short microsteps) {
  for (short ms = 1; ms <= getMaxMicrostep(); ms <<= 1) {
    if (microsteps == ms) {
      this->microsteps = microsteps;
      break;
    }
  }
  return this->microsteps;
}
/*
 * Set microstepping mode (1:divisor)
 * Allowed ranges for A4988G are 1:1 to 1:16
 * If the control pins are not connected, we recalculate the timing only
 */
short GBasic::setMicrostep(short microsteps) {
  GBasic::setMicrostepChild(microsteps);

  if (!IS_CONNECTED(mode0_pin) || !IS_CONNECTED(mode1_pin) ||
      !IS_CONNECTED(mode2_pin)) {
    return this->microsteps;
  }

  const uint8_t *ms_table = getMicrostepTable();
  size_t ms_table_size = getMicrostepTableSize();

  unsigned short i = 0;
  while (i < ms_table_size) {
    if (this->microsteps & (1 << i)) {
      uint8_t mask = ms_table[i];
      _sd.setPin(mode2_pin, mask & 4);
      _sd.setPin(mode1_pin, mask & 2);
      _sd.setPin(mode0_pin, mask & 1);
      break;
    }
    i++;
  }
  return this->microsteps;
}

/*
 * Set speed profile - CONSTANT_SPEED, LINEAR_SPEED (accelerated)
 * accel and decel are given in [full steps/s^2]
 */
void GBasic::setSpeedProfile(Mode mode, short accel, short decel) {
  profile.mode = mode;
  profile.accel = accel;
  profile.decel = decel;
}
void GBasic::setSpeedProfile(struct Profile profile) {
  this->profile = profile;
}

/*
 * Move the motor a given number of steps.
 * positive to move forward, negative to reverse
 */
void GBasic::move(long steps) {
  startMove(steps);
  while (nextAction())
    ;
}
/*
 * Move the motor a given number of degrees (1-360)
 */
void GBasic::rotate(long deg) { move(calcStepsForRotation(deg)); }
/*
 * Move the motor with sub-degree precision.
 * Note that using this function even once will add 1K to your program size
 * due to inclusion of float support.
 */
void GBasic::rotate(double deg) { move(calcStepsForRotation(deg)); }

/*
 * Set up a new move (calculate and save the parameters)
 */
void GBasic::startMove(long steps, long time) {
  float speed;
  // set up new move
  dir_state = (steps >= 0) ? HIGH : LOW;
  last_action_end = 0;
  steps_remaining = abs(steps);
  step_count = 0;
  rest = 0;
  switch (profile.mode) {
  case LINEAR_SPEED:
    // speed is in [steps/s]
    speed = rpm * motor_steps / 60;
    if (time > 0) {
      // Calculate a new speed to finish in the time requested
      float t = time / (1e+6);                // convert to seconds
      float d = steps_remaining / microsteps; // convert to full steps
      float a2 = 1.0 / profile.accel + 1.0 / profile.decel;
      float sqrt_candidate = t * t - 2 * a2 * d; // in √b^2-4ac
      if (sqrt_candidate >= 0) {
        speed = min(speed, (t - (float)sqrt(sqrt_candidate)) / a2);
      };
    }
    // how many microsteps from 0 to target speed
    steps_to_cruise = microsteps * (speed * speed / (2 * profile.accel));
    // how many microsteps are needed from cruise speed to a full stop
    steps_to_brake = steps_to_cruise * profile.accel / profile.decel;
    if (steps_remaining < steps_to_cruise + steps_to_brake) {
      // cannot reach max speed, will need to brake early
      steps_to_cruise =
          steps_remaining * profile.decel / (profile.accel + profile.decel);
      steps_to_brake = steps_remaining - steps_to_cruise;
    }
    // Initial pulse (c0) including error correction factor 0.676 [us]
    step_pulse = (1e+6) * 0.676 * sqrt(2.0f / profile.accel / microsteps);
    // Save cruise timing since we will no longer have the calculated target
    // speed later
    cruise_step_pulse = 1e+6 / speed / microsteps;
    break;

  case CONSTANT_SPEED:
  default:
    steps_to_cruise = 0;
    steps_to_brake = 0;
    step_pulse = cruise_step_pulse = STEP_PULSE(rpm, motor_steps, microsteps);
    if (time > steps_remaining * step_pulse) {
      step_pulse = (float)time / steps_remaining;
    }
  }
}
/*
 * Alter a running move by adding/removing steps
 * FIXME: This is a naive implementation and it only works well in CRUISING
 * state
 */
void GBasic::alterMove(long steps) {
  switch (getCurrentState()) {
  case ACCELERATING: // this also works but will keep the original speed target
  case CRUISING:
    if (steps >= 0) {
      steps_remaining += steps;
    } else {
      steps_remaining = max(steps_to_brake, steps_remaining + steps);
    };
    break;
  case DECELERATING:
    // would need to start accelerating again -- NOT IMPLEMENTED
    break;
  case STOPPED:
    startMove(steps);
    break;
  }
}
/*
 * Brake early.
 */
void GBasic::startBrake(void) {
  switch (getCurrentState()) {
  case CRUISING: // this applies to both CONSTANT_SPEED and LINEAR_SPEED modes
    steps_remaining = steps_to_brake;
    break;

  case ACCELERATING:
    steps_remaining = step_count * profile.accel / profile.decel;
    break;

  default:
    break; // nothing to do if already stopped or braking
  }
}
/*
 * Stop movement immediately and return remaining steps.
 */
long GBasic::stop(void) {
  long retval = steps_remaining;
  steps_remaining = 0;
  return retval;
}
/*
 * Return calculated time to complete the given move
 */
long GBasic::getTimeForMove(long steps) {
  float t;
  if (steps == 0) {
    return 0;
  }
  switch (profile.mode) {
  case LINEAR_SPEED:
    startMove(steps);
    if (steps_remaining >= steps_to_cruise + steps_to_brake) {
      float speed = rpm * motor_steps / 60; // full steps/s
      t = (steps / (microsteps * speed)) + (speed / (2 * profile.accel)) +
          (speed / (2 * profile.decel)); // seconds
    } else {
      t = sqrt(2.0 * steps_to_cruise / profile.accel / microsteps) +
          sqrt(2.0 * steps_to_brake / profile.decel / microsteps);
    }
    t *= (1e+6); // seconds -> micros
    break;
  case CONSTANT_SPEED:
  default:
    t = steps * STEP_PULSE(rpm, motor_steps, microsteps);
  }
  return round(t);
}
/*
 * Move the motor an integer number of degrees (360 = full rotation)
 * This has poor precision for small amounts, since step is usually 1.8deg
 */
void GBasic::startRotate(long deg) { startMove(calcStepsForRotation(deg)); }
/*
 * Move the motor with sub-degree precision.
 * Note that calling this function will increase program size substantially
 * due to inclusion of float support.
 */
void GBasic::startRotate(double deg) { startMove(calcStepsForRotation(deg)); }

/*
 * calculate the interval til the next pulse
 */
void GBasic::calcStepPulse(void) {
  if (steps_remaining <=
      0) { // this should not happen, but avoids strange calculations
    return;
  }
  steps_remaining--;
  step_count++;

  if (profile.mode == LINEAR_SPEED) {
    switch (getCurrentState()) {
    case ACCELERATING:
      if (step_count < steps_to_cruise) {
        step_pulse =
            step_pulse - (2 * step_pulse + rest) / (4 * step_count + 1);
        rest = (step_count < steps_to_cruise)
                   ? (2 * step_pulse + rest) % (4 * step_count + 1)
                   : 0;
      } else {
        // The series approximates target, set the final value to what it should
        // be instead
        step_pulse = cruise_step_pulse;
      }
      break;

    case DECELERATING:
      step_pulse =
          step_pulse - (2 * step_pulse + rest) / (-4 * steps_remaining + 1);
      rest = (2 * step_pulse + rest) % (-4 * steps_remaining + 1);
      break;

    default:
      break; // no speed changes
    }
  }
}
/*
 * Yield to step control
 * Toggle step and return time until next change is needed (micros)
 */
long GBasic::nextAction(void) {
  if (steps_remaining > 0) {
    delayMicros(next_action_interval, last_action_end);
    /*
     * DIR pin is sampled on rising STEP edge, so it is set first
     */
    _sd.setPin(dir_pin, dir_state);
    _sd.setPin(step_pin, HIGH);
    unsigned m = micros();
    unsigned long pulse =
        step_pulse; // save value because calcStepPulse() will overwrite it
    calcStepPulse();
    // We should pull HIGH for at least 1-2us (step_high_min)
    delayMicros(step_high_min);
    _sd.setPin(step_pin, LOW);
    // account for calcStepPulse() execution time; sets ceiling for max rpm on
    // slower MCUs
    last_action_end = micros();
    m = last_action_end - m;
    next_action_interval = (pulse > m) ? pulse - m : 1;
  } else {
    // end of move
    last_action_end = 0;
    next_action_interval = 0;
  }
  return next_action_interval;
}

enum GBasic::State GBasic::getCurrentState(void) {
  enum State state;
  if (steps_remaining <= 0) {
    state = STOPPED;
  } else {
    if (steps_remaining <= steps_to_brake) {
      state = DECELERATING;
    } else if (step_count <= steps_to_cruise) {
      state = ACCELERATING;
    } else {
      state = CRUISING;
    }
  }
  return state;
}
/*
 * Configure which logic state on ENABLE pin means active
 * when using SLEEP (default) this is active HIGH
 */
void GBasic::setEnableActiveState(short state) { enable_active_state = state; }
/*
 * Enable/Disable the motor by setting a digital flag
 */
void GBasic::enable(void) {
  if IS_CONNECTED (enable_pin) {
    _sd.setPin(enable_pin, enable_active_state);
  };
  delayMicros(2);
}

void GBasic::disable(void) {
  if IS_CONNECTED (enable_pin) {
    _sd.setPin(enable_pin, (enable_active_state == HIGH) ? LOW : HIGH);
  }
}

short GBasic::getMaxMicrostep() { return GBasic::MAX_MICROSTEP; }

const uint8_t *GBasic::getMicrostepTable() {
  return (uint8_t *)GBasic::MS_TABLE;
}

size_t GBasic::getMicrostepTableSize() { return sizeof(GBasic::MS_TABLE); }
