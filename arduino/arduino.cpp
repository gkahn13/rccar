/* Libraries */
#include "PinChangeInterrupt.h"

/* General definitions */
#define LOOP_DELAY 10
volatile unsigned long int previous_loop_time = 0;


/* Radio definitions */

#define PIN_RADIO_SERVO 11
#define PIN_RADIO_MOTOR 12
#define PIN_RADIO_BUTTON 13

#define RADIO_SERVO_MIN 1000
#define RADIO_SERVO_NEUTRAL 1500
#define RADIO_SERVO_MAX 2000

#define RADIO_MOTOR_MIN 1200
#define RADIO_MOTOR_NEUTRAL 1500
#define RADIO_MOTOR_MAX 2000

#define RADIO_BUTTON_MIN 800 // up
#define RADIO_BUTTON_MAX 2100 // down
#define RADIO_BUTTON_LOCK_PWM 2100
#define RADIO_BUTTON_RADIO_PWM 1500
#define RADIO_BUTTON_SERIAL_PWM 900

volatile unsigned long int radio_servo_curr_interrupt_time = 0;
volatile unsigned long int radio_motor_curr_interrupt_time = 0;
volatile unsigned long int radio_button_curr_interrupt_time = 0;

volatile unsigned long int radio_servo_prev_interrupt_time = 0;
volatile unsigned long int radio_motor_prev_interrupt_time = 0;
volatile unsigned long int radio_button_prev_interrupt_time = 0;

volatile unsigned short int radio_servo_pwm = RADIO_SERVO_NEUTRAL;
volatile unsigned short int radio_motor_pwm = RADIO_MOTOR_NEUTRAL;
volatile unsigned short int radio_button_pwm = RADIO_BUTTON_MIN;



/* Serial definitions */

#define SERIAL_SERVO_MIN 1
#define SERIAL_SERVO_NEUTRAL 5000
#define SERIAL_SERVO_MAX 9999
volatile unsigned short int serial_servo = SERIAL_SERVO_NEUTRAL;

#define SERIAL_MOTOR_MIN 1
#define SERIAL_MOTOR_NEUTRAL 5000
#define SERIAL_MOTOR_MAX 9999
volatile unsigned short int serial_motor = SERIAL_MOTOR_NEUTRAL;



/* Control definitions */

#define CONTROL_LOCK 0
#define CONTROL_RADIO 1
#define CONTROL_SERIAL 2
volatile unsigned short int control_mode = CONTROL_LOCK;

volatile float control_servo_pct = 0.0; // [-1, 1]
volatile float control_motor_pct = 0.0; // [-1, 1]

volatile unsigned short int control_servo_pwm = RADIO_SERVO_NEUTRAL;
volatile unsigned short int control_motor_pwm = RADIO_MOTOR_NEUTRAL;



/* Interrupts */

void interrupt_radio_servo(void) {
  radio_servo_curr_interrupt_time = micros();
  unsigned short int pwm = radio_servo_curr_interrupt_time - radio_servo_prev_interrupt_time;
  radio_servo_prev_interrupt_time = radio_servo_curr_interrupt_time;

  if ((pwm >= RADIO_SERVO_MIN) && (pwm <= RADIO_SERVO_MAX)) {
    radio_servo_pwm = pwm;
  }
}

void interrupt_radio_motor(void) {
  radio_motor_curr_interrupt_time = micros();
  unsigned short int pwm = radio_motor_curr_interrupt_time - radio_motor_prev_interrupt_time;
  radio_motor_prev_interrupt_time = radio_motor_curr_interrupt_time;

  if ((pwm >= RADIO_MOTOR_MIN) && (pwm <= RADIO_MOTOR_MAX)) {
    radio_motor_pwm = pwm;
  }
}

void interrupt_radio_button(void) {
  // get radio pwm
  radio_button_curr_interrupt_time = micros();
  unsigned short int pwm = radio_button_curr_interrupt_time - radio_button_prev_interrupt_time;
  radio_button_prev_interrupt_time = radio_button_curr_interrupt_time;

  if ((pwm >= RADIO_BUTTON_MIN) && (pwm <= RADIO_BUTTON_MAX)) {
    radio_button_pwm = pwm;
  }

  // change control mode
  unsigned short int lock_pwm_diff = radio_button_pwm > RADIO_BUTTON_LOCK_PWM ? radio_button_pwm - RADIO_BUTTON_LOCK_PWM : RADIO_BUTTON_LOCK_PWM - radio_button_pwm;
  unsigned short int radio_pwm_diff = radio_button_pwm > RADIO_BUTTON_RADIO_PWM ? radio_button_pwm - RADIO_BUTTON_RADIO_PWM : RADIO_BUTTON_RADIO_PWM - radio_button_pwm;
  unsigned short int serial_pwm_diff = radio_button_pwm > RADIO_BUTTON_SERIAL_PWM ? radio_button_pwm - RADIO_BUTTON_SERIAL_PWM : RADIO_BUTTON_SERIAL_PWM - radio_button_pwm;

  if ((radio_pwm_diff <= lock_pwm_diff) && (radio_pwm_diff <= serial_pwm_diff)) {
    control_mode = CONTROL_RADIO;
  } else if ((serial_pwm_diff <= lock_pwm_diff) && (serial_pwm_diff <= radio_pwm_diff)){
    if (control_mode != CONTROL_SERIAL) {
      serial_servo = SERIAL_SERVO_NEUTRAL;
      serial_motor = SERIAL_MOTOR_NEUTRAL;
    }
    control_mode = CONTROL_SERIAL;
  } else {
    control_mode = CONTROL_LOCK;
  }
}



/* Serial parsing */

void serial_parse(void) {
  /*
   * Assumes long to parse is aaaabbbb, where
   * a - servo [0, 9999]
   * b - motor [0, 9999]
   */
  // long parsed_int = Serial.parseInt();
  // if (parsed_int > 0) {
  //   serial_servo = parsed_int / 10000; // 1e4
  //   parsed_int -= serial_servo * 10000;
  //   serial_motor = parsed_int;
  // }

  String parsed_str = Serial.readStringUntil('\n');
  parsed_str.trim();
  unsigned short int length = parsed_str.length();

  long parsed_int = 0;
  if (length >= 3) {
    if ((parsed_str.charAt(0) == '(') && (parsed_str.charAt(length - 1) == ')')) {
      parsed_int = parsed_str.substring(1, length - 1).toInt();
    }
  }
  if (parsed_int > 0) {
    serial_servo = parsed_int / 10000; // 1e4
    parsed_int -= serial_servo * 10000;
    serial_motor = parsed_int;
  }
}

void serial_write(void) {
  /*
   * Writes tuple (control mode, servo, motor)
   */

  Serial.print("(");
  Serial.print(control_mode);
  Serial.print(",");
  Serial.print(control_servo_pct, 4);
  Serial.print(",");
  Serial.print(control_motor_pct, 4);
  Serial.println(")");
}



/* Control loop */

void control_loop(void) {
  // convert to pct
  unsigned short int pwm = 0;
  if (control_mode == CONTROL_RADIO) {
    // servo
    pwm = radio_servo_pwm;
    if (pwm >= RADIO_SERVO_NEUTRAL) {
      control_servo_pct = ((float) (pwm - RADIO_SERVO_NEUTRAL)) / (RADIO_SERVO_MAX - RADIO_SERVO_NEUTRAL);
    } else {
      control_servo_pct = -((float) (RADIO_SERVO_NEUTRAL - pwm)) / (RADIO_SERVO_NEUTRAL - RADIO_SERVO_MIN);
    }

    // motor
    pwm = radio_motor_pwm;
    if (pwm >= RADIO_MOTOR_NEUTRAL) {
      control_motor_pct = ((float) (pwm - RADIO_MOTOR_NEUTRAL)) / (RADIO_MOTOR_MAX - RADIO_MOTOR_NEUTRAL);
    } else {
      control_motor_pct = -((float) (RADIO_MOTOR_NEUTRAL - pwm)) / (RADIO_MOTOR_NEUTRAL - RADIO_MOTOR_MIN);
    }
  } else if (control_mode == CONTROL_SERIAL) {
    // parse serial
    serial_parse();

    // convert serial
    if (serial_servo >= SERIAL_SERVO_NEUTRAL) {
      control_servo_pct = ((float) (serial_servo - SERIAL_SERVO_NEUTRAL)) / (SERIAL_SERVO_MAX - SERIAL_SERVO_NEUTRAL);
    } else {
      control_servo_pct = -((float) (SERIAL_SERVO_NEUTRAL - serial_servo)) / (SERIAL_SERVO_NEUTRAL - SERIAL_SERVO_MIN);
    }

    if (serial_motor >= SERIAL_MOTOR_NEUTRAL) {
      control_motor_pct = ((float) (serial_motor - SERIAL_MOTOR_NEUTRAL)) / (SERIAL_MOTOR_MAX - SERIAL_MOTOR_NEUTRAL);
    } else {
      control_motor_pct = - ((float) (SERIAL_MOTOR_NEUTRAL - serial_motor)) / (SERIAL_MOTOR_NEUTRAL - SERIAL_MOTOR_MIN);
    }
  } else {
    control_servo_pct = 0.0;
    control_motor_pct = 0.0;
  }

  // convert to pwm
  if (control_servo_pct >= 0) {
    control_servo_pwm = control_servo_pct * (RADIO_SERVO_MAX - RADIO_SERVO_NEUTRAL) + RADIO_SERVO_NEUTRAL;
  } else {
    control_servo_pwm = control_servo_pct * (RADIO_SERVO_NEUTRAL - RADIO_SERVO_MIN) + RADIO_SERVO_NEUTRAL;
  }

  if (control_motor_pct >= 0) {
    control_motor_pwm = control_motor_pct * (RADIO_MOTOR_MAX - RADIO_MOTOR_NEUTRAL) + RADIO_MOTOR_NEUTRAL;
  } else {
    control_motor_pwm = control_motor_pct * (RADIO_MOTOR_NEUTRAL - RADIO_MOTOR_MIN) + RADIO_MOTOR_NEUTRAL;
  }
}



/* Main */

void setup() {
  // Serial
  Serial.begin(115200);
  Serial.setTimeout(5);

  // Radio interrupts
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_RADIO_SERVO),
    interrupt_radio_servo, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_RADIO_MOTOR),
    interrupt_radio_motor, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_RADIO_BUTTON),
    interrupt_radio_button, CHANGE);
}

void loop() {
  unsigned long int current_loop_time = millis();

  if (current_loop_time - previous_loop_time >= LOOP_DELAY) {
    previous_loop_time = current_loop_time;

    control_loop();
    serial_write();
  }
}
