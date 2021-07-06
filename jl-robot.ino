#include "src/AccelStepper/src/AccelStepper.h"
#include "src/AccelStepper/src/MultiStepper.h"
#include "src/DigitLedDisplay/src/DigitLedDisplay.h"
#include "src/Bounce2/src/Bounce2.h"

// Set up physical parameters
# define pulley_teeth 20
# define belt_pitch_mm 2
# define steps_per_rev 200
# define pump_pitch_mm 1
# define microstep 4
// CHANGE THIS VALUE FOR DIFFERENT SYRINGE TYPES
# define p_mm_per_ul 1.064

# define p_invert 1

# define xmin 0.0
# define xmax 320.0
# define xstep 10.0

# define pmin 0.0
# define pmax 100.0
# define pstep 1.0

# define tmin 10
# define tmax 1000
# define tstep 1


# define x_steps_per_mm -microstep*(steps_per_rev)/(pulley_teeth*belt_pitch_mm)
# define p_steps_per_mm p_invert*microstep*(steps_per_rev)/(pump_pitch_mm)

// Set up pin definitions
# define GO_BTN A2
# define PARAM_SEL A0
# define VALUE_UP 11
# define VALUE_DOWN 10
# define RESET_BTN A1

# define EN 8
# define X_DIR 5
# define P_DIR 6
# define X_STP 2
# define P_STP 3

# define DIN 13
# define CS 12
# define CLK 9

# define motorInterfaceType 2

DigitLedDisplay ld = DigitLedDisplay(DIN, CS, CLK);

AccelStepper x_motor = AccelStepper(motorInterfaceType, X_STP, X_DIR);
AccelStepper p_motor = AccelStepper(motorInterfaceType, P_STP, P_DIR);

MultiStepper motors;

Bounce go_btn = Bounce();
Bounce param_sel = Bounce();
Bounce value_up = Bounce();
Bounce value_down = Bounce();
Bounce reset_btn = Bounce();

long p = 10;
long x = 300;
float t = 20;// time in seconds x 10
long x_speed = 2000;
long p_speed = 2000;

long xcurr = 0;
long pcurr = 0;

long positions[2];

int param = 2;

bool reset = false;

void setup() {
  Serial.begin(9600);
  
  x_motor.setEnablePin(EN);
  p_motor.setEnablePin(EN);

  x_motor.setMaxSpeed(3600);
  p_motor.setMaxSpeed(4000);
  
  x_motor.setSpeed(x_speed);
  x_motor.setCurrentPosition(0);
  x_motor.setAcceleration(100);

  p_motor.setSpeed(p_speed);
  p_motor.setCurrentPosition(0);
  p_motor.setAcceleration(100);

  motors.addStepper(x_motor);
  motors.addStepper(p_motor);

  ld.setBright(15);
  ld.setDigitLimit(8);

  go_btn.attach(GO_BTN, INPUT_PULLUP);
  param_sel.attach(PARAM_SEL, INPUT_PULLUP);
  value_up.attach(VALUE_UP, INPUT_PULLUP);
  value_down.attach(VALUE_DOWN, INPUT_PULLUP);
  reset_btn.attach(RESET_BTN, INPUT_PULLUP);
  
  go_btn.interval(20);
  param_sel.interval(20);
  value_up.interval(20);
  value_down.interval(20);
  reset_btn.interval(20);
}

void loop() {
  go_btn.update();
  param_sel.update();
  value_up.update();
  value_down.update();
  reset_btn.update();

  if (param_sel.fell()) {
    ld.clear();
    param++;
    if (param > 2) {
      param = 0;
    }
  }

  if (value_up.fell() and reset == false) {
    ld.clear();
    switch (param) {
      case 0:
        x = x + xstep;
        if (x > xmax) {x = xmax;}
        break;
      case 1:
        p = p + pstep;
        if (p > pmax) {p = pmax;}
        break;
      case 2:
        t = t + tstep;
        if (t > tmax) {t = tmax;}
        break;
    }
  }

  if (value_down.fell() and reset == false) {
    ld.clear();
    switch (param) {
      case 0:
        x = x - xstep;
        if (x < xmin) {x = xmin;}
        break;
      case 1:
        p = p - pstep;
        if (p < pmin) {p = pmin;}
        break;
      case 2:
        t = t - tstep;
        if (t < tmin) {t = tmin;}
        break;
    }
  }

  switch (param) {
    case 0:
      ld.printDigit(x);
      //Loc
      ld.write(8, B00001110);
      ld.write(7, B00011101);
      ld.write(6, B00001101);
      ld.write(5, B00000000);
      break;
    case 1:
      ld.printDigit(p);
      //Vol
      ld.write(8, B00111110);
      ld.write(7, B00011101);
      ld.write(6, B00110000);
      ld.write(5, B00000000);
      break;
    case 2:
      ld.write(2, B10000000);
      ld.printDigit(t);
      //Velo
      ld.write(8, B00111110);
      ld.write(7, B01001111);
      ld.write(6, B00110000);
      ld.write(5, B00011101);
      break;
  }
  
  if (go_btn.fell()) {
    if (reset == true) {
      positions[0] = 0;
      motors.moveTo(positions);
      reset = false;
    }
    else {
      positions[0] = (x_steps_per_mm*x);
      x_speed = x_steps_per_mm*(x)/(t/10);
      x_motor.setMaxSpeed(abs(x_speed));

      pcurr = p+pcurr;
      positions[1] = (p_steps_per_mm*p_mm_per_ul*(pcurr));
      p_speed = p_steps_per_mm*p_mm_per_ul*(p)/(t/10);
      p_motor.setMaxSpeed(abs(p_speed));
      
      MultiStepper motors;
      motors.addStepper(x_motor);
      motors.addStepper(p_motor);

      motors.moveTo(positions);

      reset = true;
    }
    motors.runSpeedToPosition();
  } 
  if (reset_btn.fell()) {
    if (p_invert == -1) { digitalWrite(P_DIR, HIGH); }
    else if (p_invert == -1) { digitalWrite(P_DIR, LOW); }
    while (reset_btn.read() == LOW) {
      digitalWrite(P_STP, LOW);
      delayMicroseconds(500);
      digitalWrite(P_STP, HIGH);
      delayMicroseconds(500);
      reset_btn.update();
    }
  }

}
