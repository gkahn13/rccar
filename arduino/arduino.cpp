/*
 * Libraries
 */
#include "PinChangeInterrupt.h"
#include <Servo.h>
#include <TimerOne.h>

/***************
 * Definitions *
 ***************/

/*
 * General definitions
 */
#define LOOP_DELAY 10
volatile unsigned long int current_loop_time = 0;
volatile unsigned long int previous_loop_time = 0;
volatile unsigned long int delay_time = 0;


/*
 * Radio definitions
 */

#define PIN_RADIO_SERVO 13
#define PIN_RADIO_MOTOR 12
#define PIN_RADIO_BUTTON 11

#define RADIO_SERVO_MIN 1000
#define RADIO_SERVO_NEUTRAL 1500
#define RADIO_SERVO_MAX 2000

#define RADIO_MOTOR_MIN 1200
#define RADIO_MOTOR_NEUTRAL 1500
#define RADIO_MOTOR_MAX 2000

#define RADIO_BUTTON_MIN 100 // up
#define RADIO_BUTTON_MAX 10000 // down
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



/*
 * Serial definitions
 */

#define SERIAL_BAUD_RATE 115200

#define SERIAL_SERVO_MIN 1
#define SERIAL_SERVO_NEUTRAL 5000
#define SERIAL_SERVO_MAX 9999
volatile unsigned short int serial_servo = SERIAL_SERVO_NEUTRAL;

#define SERIAL_MOTOR_MIN 1
#define SERIAL_MOTOR_NEUTRAL 5000
#define SERIAL_MOTOR_MAX 9999
volatile unsigned short int serial_motor = SERIAL_MOTOR_NEUTRAL;

#define START_BYTE '<'
#define STOP_BYTE '>'


/*
 * Control definitions
 */

#define CONTROL_LOCK 0
#define CONTROL_RADIO 1
#define CONTROL_SERIAL 2
volatile unsigned short int control_mode = CONTROL_LOCK;

volatile float control_servo_pct = 0.0; // [-1, 1]
volatile float control_motor_pct = 0.0; // [-1, 1]

volatile unsigned short int control_servo_pwm = RADIO_SERVO_NEUTRAL;
volatile unsigned short int control_motor_pwm = RADIO_MOTOR_NEUTRAL;

#define PIN_SERVO 10
#define PIN_MOTOR 9

Servo servo;
Servo motor;

/*
 * Battery definitions
 */

#define PIN_BATT_A 0
#define PIN_BATT_B 1

const float BATT_RATIO = 12.6 / 3.94;

volatile float batt_a_voltage = 0.0;
volatile float batt_b_voltage = 0.0;

#define BATT_DELAY 10000 // ms
volatile unsigned long int current_batt_time = 0;
volatile unsigned long int previous_batt_time = 0;

/*
 * Encoder definitions
 */

#include "RunningAverage.h"

#define PIN_ENCODER_LEFT 3
#define PIN_ENCODER_RIGHT 2
#define INTERRUPT_ENCODER_LEFT 1
#define INTERRUPT_ENCODER_RIGHT 0

#define ENCODER_AVG_LEN 12
RunningAverage encoder_avg_times_left(ENCODER_AVG_LEN);
RunningAverage encoder_avg_times_right(ENCODER_AVG_LEN);

const float ENCODER_CONST = (1000.0 * 1000.0) * (0.04 * 3.141592654) / (6.0);
const float ENCODER_MIN_DT = 1000000.0 * 0.14;

volatile unsigned long int encoder_curr_time_left = 0;
volatile unsigned long int encoder_prev_time_left = 0;
volatile unsigned long int encoder_curr_time_right = 0;
volatile unsigned long int encoder_prev_time_right = 0;

volatile float encoder_rate_left = 0.0;
volatile float encoder_rate_right = 0.0;


/*
 * IMU definitions
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Madgwick.h>


#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

#define IMU_DELAY 10 // ms

// Create sensor instances.
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each board/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -272.47F, -124.95F, -100.16F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.975,  0.040,  -0.008 },
                                    {  0.040,  1.032, -0.009 },
                                    {  -0.008, -0.009,  0.996 } };

float mag_field_strength        = 56.67F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Mahony is lighter weight as a filter and should be used
// on slower systems
Mahony filter;
//Madgwick filter;

// accel
volatile float imu_accel_x = 0.0;
volatile float imu_accel_y = 0.0;
volatile float imu_accel_z = 0.0;

// gyro
volatile float imu_gyro_x = 0.0;
volatile float imu_gyro_y = 0.0;
volatile float imu_gyro_z = 0.0;

// orientation
volatile float imu_q0 = 1.0;
volatile float imu_q1 = 0.0;
volatile float imu_q2 = 0.0;
volatile float imu_q3 = 0.0;


/*************
 * Functions *
 *************/

/*
 * Radio functions
 */

void setup_radio(void) {
  // Radio interrupts
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_RADIO_SERVO),
    interrupt_radio_servo, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_RADIO_MOTOR),
    interrupt_radio_motor, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_RADIO_BUTTON),
    interrupt_radio_button, CHANGE);
}

void interrupt_radio_servo(void) {
  radio_servo_curr_interrupt_time = micros();
  if (!(radio_servo_curr_interrupt_time > radio_servo_prev_interrupt_time)) {
    return;
  }
  unsigned short int pwm = radio_servo_curr_interrupt_time - radio_servo_prev_interrupt_time;
  radio_servo_prev_interrupt_time = radio_servo_curr_interrupt_time;

  if ((pwm >= RADIO_SERVO_MIN) && (pwm <= RADIO_SERVO_MAX)) {
    radio_servo_pwm = pwm;
  }
}

void interrupt_radio_motor(void) {
  radio_motor_curr_interrupt_time = micros();
  if (!(radio_motor_curr_interrupt_time > radio_motor_prev_interrupt_time)) {
    return;
  }
  unsigned short int pwm = radio_motor_curr_interrupt_time - radio_motor_prev_interrupt_time;
  radio_motor_prev_interrupt_time = radio_motor_curr_interrupt_time;

  if ((pwm >= RADIO_MOTOR_MIN) && (pwm <= RADIO_MOTOR_MAX)) {
    radio_motor_pwm = pwm;
  }
}

void interrupt_radio_button(void) {
  // get radio pwm
  radio_button_curr_interrupt_time = micros();
  if (!(radio_button_curr_interrupt_time > radio_button_prev_interrupt_time)) {
    return;
  }
  unsigned long int pwm = radio_button_curr_interrupt_time - radio_button_prev_interrupt_time;
  radio_button_prev_interrupt_time = radio_button_curr_interrupt_time;

  if ((pwm >= RADIO_BUTTON_MIN) && (pwm <= RADIO_BUTTON_MAX)) {
    radio_button_pwm = pwm;
  }

  // change control mode
  unsigned long int lock_pwm_diff = radio_button_pwm > RADIO_BUTTON_LOCK_PWM ? radio_button_pwm - RADIO_BUTTON_LOCK_PWM : RADIO_BUTTON_LOCK_PWM - radio_button_pwm;
  unsigned long int radio_pwm_diff = radio_button_pwm > RADIO_BUTTON_RADIO_PWM ? radio_button_pwm - RADIO_BUTTON_RADIO_PWM : RADIO_BUTTON_RADIO_PWM - radio_button_pwm;
  unsigned long int serial_pwm_diff = radio_button_pwm > RADIO_BUTTON_SERIAL_PWM ? radio_button_pwm - RADIO_BUTTON_SERIAL_PWM : RADIO_BUTTON_SERIAL_PWM - radio_button_pwm;

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



/*
 * Serial functions
 */

void setup_serial(void){
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(5);
}

void serial_parse(void) {
  /*
   * Assumes long to parse is (aaaabbbb), where
   * a - servo [0, 9999]
   * b - motor [0, 9999]
   */

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

serial_write_float(float f) {
  byte * b = (byte *) &f;
  Serial.write(b[0]);
  Serial.write(b[1]);
  Serial.write(b[2]);
  Serial.write(b[3]);
}

void serial_write(void) {
  Serial.print(START_BYTE);
  serial_write_float(((float)control_mode));
  serial_write_float(control_servo_pct);
  serial_write_float(control_motor_pct);
  serial_write_float(batt_a_voltage);
  serial_write_float(batt_b_voltage);
  serial_write_float(encoder_rate_left);
  serial_write_float(encoder_rate_right);
  serial_write_float(imu_q0);
  serial_write_float(imu_q1);
  serial_write_float(imu_q2);
  serial_write_float(imu_q3);
  serial_write_float(imu_accel_x);
  serial_write_float(imu_accel_y);
  serial_write_float(imu_accel_z);
  serial_write_float(imu_gyro_x);
  serial_write_float(imu_gyro_y);
  serial_write_float(imu_gyro_z);
  Serial.write(STOP_BYTE);

  // Serial.print("(");

  // Serial.print(control_mode);
  // Serial.print(",");

  // Serial.print(control_servo_pct, 4);
  // Serial.print(",");
  // Serial.print(control_motor_pct, 4);
  // Serial.print(",");

  // Serial.print(batt_a_voltage, 2);
  // Serial.print(",");
  // Serial.print(batt_b_voltage, 2);
  // Serial.print(",");

  // Serial.print(encoder_rate_left, 5);
  // Serial.print(",");
  // Serial.print(encoder_rate_right, 5);
  // Serial.print(",");

  // uint8_t count_left = encoder_avg_times_left.getCount();
  // Serial.print((1000.0 * 1000.0 * count_left) / (16.0 * (count_left * encoder_avg_times_left.getAverage())));
  // Serial.print(",");

  // Serial.print("(");
  // Serial.print(imu_q0, 5);
  // Serial.print(",");
  // Serial.print(imu_q1, 5);
  // Serial.print(",");
  // Serial.print(imu_q2, 5);
  // Serial.print(",");
  // Serial.print(imu_q3, 5);
  // Serial.print(")");
  // Serial.print(",");

  // Serial.print("(");
  // Serial.print(imu_accel_x, 5);
  // Serial.print(",");
  // Serial.print(imu_accel_y, 5);
  // Serial.print(",");
  // Serial.print(imu_accel_z, 5);
  // Serial.print(")");
  // Serial.print(",");

  // Serial.print("(");
  // Serial.print(imu_gyro_x, 5);
  // Serial.print(",");
  // Serial.print(imu_gyro_y, 5);
  // Serial.print(",");
  // Serial.print(imu_gyro_z, 5);
  // Serial.print(")");

  // Serial.println(")");
}



/*
 * Control functions
 */

void setup_control(void) {
  servo.attach(PIN_SERVO);
  motor.attach(PIN_MOTOR);
}

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

  // write pwm
  servo.writeMicroseconds(control_servo_pwm);
  motor.writeMicroseconds(control_motor_pwm);
}


/*
 * Battery functions
 */

void setup_battery(void) {
}

float get_voltage(int adc_pin) {
  analogRead(adc_pin);
  int reading = 0;
  for(int i=0; i < 5; ++i) {
    reading += analogRead(adc_pin);
    delay(20);
  }
  float voltage = BATT_RATIO * 5.0 * (reading / 1023.0);
  voltage /= 5; // for averaging
  return voltage;
}

void battery_loop(void) {
  batt_a_voltage = get_voltage(PIN_BATT_A);
  batt_b_voltage = get_voltage(PIN_BATT_B);
}


/*
 * Encoder functions
 */

void setup_encoder(void) {
  pinMode(PIN_ENCODER_LEFT, INPUT);
  pinMode(PIN_ENCODER_RIGHT, INPUT);

  attachInterrupt(INTERRUPT_ENCODER_LEFT, interrupt_encoder_left, RISING);
  attachInterrupt(INTERRUPT_ENCODER_RIGHT, interrupt_encoder_right, RISING);
}

void interrupt_encoder_left(void) {
  encoder_curr_time_left = micros();
  if (encoder_prev_time_left > 0) {
    encoder_avg_times_left.addValue(encoder_curr_time_left - encoder_prev_time_left);
  }
  encoder_prev_time_left = encoder_curr_time_left;
}

void interrupt_encoder_right(void) {
  encoder_curr_time_right = micros();
  if (encoder_prev_time_right > 0) {
    encoder_avg_times_right.addValue(encoder_curr_time_right - encoder_prev_time_right);
  }
  encoder_prev_time_right = encoder_curr_time_right;
}

void encoder_loop(void) {
  uint8_t count_left = encoder_avg_times_left.getCount();
  if (count_left > 0) {
    unsigned long int added_time_left = micros() - encoder_prev_time_left;
    float encoder_avg_left = encoder_avg_times_left.getAverage();
    if (added_time_left > ENCODER_MIN_DT) {
      encoder_rate_left = 0.0;
      encoder_avg_times_left.clear();
    } else {
      encoder_rate_left = (ENCODER_CONST * count_left) / (count_left * encoder_avg_left + added_time_left);
    }
  } else {
    encoder_rate_left = 0.0;
  }

  uint8_t count_right = encoder_avg_times_right.getCount();
  if (count_right > 0) {
    unsigned long int added_time_right = micros() - encoder_prev_time_right;
    float encoder_avg_right = encoder_avg_times_right.getAverage();
    if (added_time_right > ENCODER_MIN_DT) {
      encoder_rate_right = 0.0;
      encoder_avg_times_right.clear();
    } else {
      encoder_rate_right = (ENCODER_CONST * count_right) / (count_right * encoder_avg_times_right.getAverage() + added_time_right);
    }
  } else {
    encoder_rate_right = 0.0;
  }

}

/*
 * IMU functions
 */

void setup_imu(void) {
  if(!gyro.begin()) {
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
    while(1);
  }

  if(!accelmag.begin(ACCEL_RANGE_4G)) {
    Serial.println("Ooops, no accel detected ... Check your wiring!");
    while(1);
  }

  // Filter expects 70 samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(IMU_DELAY);

  // Timer1.initialize(IMU_DELAY * 1000); // us
  // Timer1.attachInterrupt(interrupt_imu, IMU_DELAY * 1000); // us
}

void imu_loop(void)
{
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  // Get new data samples
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // read gyro now so it's in rad/s
  imu_gyro_x = gx;
  imu_gyro_y = gy;
  imu_gyro_z = gz;

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  // read accelerometer
  imu_accel_x = accel_event.acceleration.x;
  imu_accel_y = accel_event.acceleration.y;
  imu_accel_z = accel_event.acceleration.z;

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  // Read the filter
  float q0, q1, q2, q3;
  filter.getQuaternion(&q0, &q1, &q2, &q3);
  imu_q0 = q0;
  imu_q1 = q1;
  imu_q2 = q2;
  imu_q3 = q3;
}





/*
 * Main
 */

void setup() {
  setup_serial();
  setup_radio();
  setup_control();
  setup_battery();
  setup_encoder();
  setup_imu();

  battery_loop();
}

void loop() {

  current_loop_time = millis();
  if (current_loop_time - previous_loop_time >= LOOP_DELAY) {
    previous_loop_time = current_loop_time;

    imu_loop();
    encoder_loop();
    control_loop();
    serial_write();

    current_batt_time = millis();
    if (current_batt_time - previous_batt_time >= BATT_DELAY) {
      previous_batt_time = current_batt_time;
      battery_loop();
    }

  }

}
