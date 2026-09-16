
# DC Motor Control with Encoder, L298N, and PID

This guide walks through the motor-control system step by step:

1. Read the encoder
2. Convert encoder pulses into position
3. Control motor position with PID
4. Calculate motor speed
5. Control motor speed with PID
6. Track a sinusoidal position target

The examples assume an Arduino, a quadrature encoder, and an L298N motor driver.

---

# 1. Hardware

## Arduino Connections

### Encoder

| Encoder | Arduino |
|---|---:|
| Channel A | Pin 2 |
| Channel B | Pin 3 |
| VCC | 5V |
| GND | GND |

Pins 2 and 3 are used because they support external interrupts on common Arduino boards such as the Uno/Nano.

### L298N

| L298N | Arduino |
|---|---:|
| ENA | Pin 5 |
| IN1 | Pin 9 |
| IN2 | Pin 10 |
| GND | GND |

Pin 5 is used for PWM speed control.

The L298N direction is controlled with IN1 and IN2:

| IN1 | IN2 | Motor |
|---|---|---|
| LOW | LOW | Stop |
| HIGH | LOW | Forward |
| LOW | HIGH | Reverse |
| HIGH | HIGH | Brake |

> Make sure the Arduino, encoder, and motor-driver logic share a common GND.

---

# 2. Reading the Encoder

A quadrature encoder has two signals:

- Channel A
- Channel B

The two signals are phase shifted relative to each other.

By looking at channel B whenever channel A changes, we can determine the direction of rotation.

For example:

````text
Forward:

A: __|‾‾|__|‾‾|__
B: ____|‾‾|__|‾‾

Reverse:

B: __|‾‾|__|‾‾|__
A: ____|‾‾|__|‾‾
````

The encoder interrupt is triggered by channel A.

## Basic Encoder Code

````ino
#define ENCA 2
#define ENCB 3

volatile long encoderPosition = 0;

void setup() {

  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENCA),
    readEncoder,
    RISING
  );
}

void loop() {

  Serial.println(encoderPosition);

  delay(100);
}

void readEncoder() {

  int b = digitalRead(ENCB);

  if (b == HIGH) {
    encoderPosition++;
  }
  else {
    encoderPosition--;
  }
}
````

## What to expect

Rotate the wheel in one direction:

````text
0
1
2
3
4
5
...
````

Rotate it in the opposite direction:

````text
5
4
3
2
1
0
-1
...
````

The encoder position can therefore be positive or negative.

If the direction is backwards from what you want, swap the `++` and `--` operations:

````ino

if (b == HIGH) {
  encoderPosition--;
}
else {
  encoderPosition++;
}
````

---

# 3. Reading Encoder Position Safely

The encoder position is modified inside an interrupt:

````ino

volatile long encoderPosition = 0;
````

Because the interrupt can change the variable while the main program is reading it, it is good practice to make an atomic copy.

On AVR-based Arduino boards, we can use:

````ino

#include <util/atomic.h>
````

Then:

````ino

long position;

ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  position = encoderPosition;
}
````

This prevents the interrupt from modifying `encoderPosition` while it is being copied.

## Encoder Position Code

````ino

#include <util/atomic.h>

#define ENCA 2
#define ENCB 3

volatile long encoderPosition = 0;

void setup() {

  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENCA),
    readEncoder,
    RISING
  );
}

void loop() {

  long position;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    position = encoderPosition;
  }

  Serial.print("Position: ");
  Serial.println(position);

  delay(100);
}

void readEncoder() {

  int b = digitalRead(ENCB);

  if (b == HIGH) {
    encoderPosition++;
  }
  else {
    encoderPosition--;
  }
}
````

At this point, the Arduino knows the relative position of the wheel.

----

# 4. Understanding Encoder Counts

The encoder gives us counts rather than physical units.

For example, suppose the encoder produces:

````text

600 counts / revolution
````

Then:

````text

0 counts      = 0 revolutions
300 counts    = 0.5 revolutions
600 counts    = 1 revolution
1200 counts   = 2 revolutions
````

The relationship is:

````text

revolutions = encoderCounts / countsPerRevolution
````

For example:

````ino

float revolutions =
    position / countsPerRevolution;
````

If you know the wheel circumference, you can also calculate distance.

````text
distance = revolutions × wheel circumference
````

For example:

````ino

float distance =
    revolutions * wheelCircumference;
````

The important thing is to determine what one encoder count represents for your particular motor/encoder combination.

----


# 5. Position Control

Now that we can measure position, we can control it.

The basic idea is:

````text

Target Position
       |
       v
     [ PID ]
       |
       v
  Motor Command
       |
       v
     Motor
       |
       v
    Encoder
       |
       +----------> Actual Position
                         |
                         +----> PID
````

The PID controller compares the desired position to the actual encoder position.

The error is:

````text

error = target - actual
````

For example:

````text

Target = 1000
Actual = 700

Error = 300
````

The motor should move forward.

If:

````text

Target = 1000
Actual = 1200

Error = -200
````

The motor should move backward.

----

# 6. L298N Motor Control

Before adding PID, we need to be able to command the motor.

The L298N uses:
* ENA for PWM
* IN1 and IN2 for direction

````ino
#define PWM_PIN 5
#define IN1 9
#define IN2 10
````

A useful function is:

````ino

void setMotor(float command) {

  if (command > 0) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    analogWrite(PWM_PIN, (int)command);
  }

  else if (command < 0) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(PWM_PIN, (int)(-command));
  }

  else {

    analogWrite(PWM_PIN, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
````

The motor command ranges from:

````text

-255 → full speed reverse
   0 → stopped
+255 → full speed forward
````


The sign controls direction.

The magnitude controls PWM.

----


# 7. Position PID Controller

A PID controller has three terms:

````text

P = proportional
I = integral
D = derivative
````

The general equation is:

````text

output = Kp × error
       + Ki × integral(error)
       + Kd × derivative(error)
````

For position control:

````text

error = targetPosition - encoderPosition
````

The proportional term is:

````ino

Kp * error
````

The integral term is:

````ino

Ki * integral
````

The derivative term is:

````ino

Kd * derivative
````

---

# 8. Basic Position PID Code

````ino

#include <util/atomic.h>

#define ENCA 2
#define ENCB 3

#define PWM_PIN 5
#define IN1 9
#define IN2 10

volatile long encoderPosition = 0;

// PID gains
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

// Desired position
long targetPosition = 1000;

// PID variables
float integral = 0;
float previousError = 0;

unsigned long previousTime;

void setup() {

  Serial.begin(115200);

  // Encoder
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENCA),
    readEncoder,
    RISING
  );

  // Motor
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  setMotor(0);

  previousTime = micros();
}

void loop() {

  // Read encoder position safely
  long position;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    position = encoderPosition;
  }

  // Calculate elapsed time
  unsigned long currentTime = micros();

  float dt =
      (currentTime - previousTime) / 1000000.0;

  previousTime = currentTime;

  if (dt <= 0) {
    return;
  }

  // Position error
  float error =
      targetPosition - position;

  // Integral
  integral += error * dt;

  // Derivative
  float derivative =
      (error - previousError) / dt;

  // PID output
  float output =
      Kp * error +
      Ki * integral +
      Kd * derivative;

  previousError = error;

  // Limit motor command
  output = constrain(output, -255, 255);

  // Drive motor
  setMotor(output);

  // Serial Plotter
  Serial.print("Target:");
  Serial.print(targetPosition);

  Serial.print("\tActual:");
  Serial.println(position);

  delay(10);
}

void readEncoder() {

  int b = digitalRead(ENCB);

  if (b == HIGH) {
    encoderPosition++;
  }
  else {
    encoderPosition--;
  }
}

void setMotor(float command) {

  if (command > 0) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    analogWrite(
      PWM_PIN,
      (int)command
    );
  }

  else if (command < 0) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(
      PWM_PIN,
      (int)(-command)
    );
  }

  else {

    analogWrite(PWM_PIN, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
````

---

# 9. Tuning Position PID

Start with:

````ino

float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
````

Do not immediately add a large integral gain.

## Step 1: Tune Kp

Increase Kp until the motor moves toward the target quickly.

If Kp is too low:

````text

Slow response
Large position error
````

If Kp is too high:

````text

Oscillation
Overshoot
Possible instability
````

## Step 2: Add Kd

If the motor overshoots or oscillates, add some derivative gain.

For example:

````ino

float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.05;
````

Increase Kd gradually.

## Step 3: Add Ki if necessary

Integral control can remove steady-state error.

For example, if the motor consistently stops slightly before the target, Ki may help.

Start very small:

````ino

float Ki = 0.01;
````

Be careful with integral windup.

---

# 10. Tracking a Sinusoidal Position

Instead of using a fixed target:

````ino

long targetPosition = 1000;
````

we can generate a continuously changing target.

The sine equation is:

````text

target = center + amplitude × sin(2πft)
````

Where:

````text

center    = center position
amplitude = maximum distance from center
f         = frequency
t         = time
````

For example:

````ino

float sineCenter = 0;
float sineAmplitude = 500;
float sineFrequency = 0.1;
````

This creates:

````text

        +500
          |
          |      /‾‾\
          |     /    \
          |----/------\----/------\----> time
          |  /          \/
          | /
        -500
````

The target moves between:

````text

-500 and +500 encoder counts
````

---

# 11. Serial Plotter

To see the target and actual position together, print them as separate signals.

````ino

Serial.print("Target:");
Serial.print(targetPosition);

Serial.print("\tActual:");
Serial.println(position);
````

The Arduino Serial Plotter should show:

````text

Target
Actual
````

as two separate traces.

Open:

````text

Tools → Serial Plotter
````

and use:

````text

115200 baud
````

Do not print other debugging information while using the Serial Plotter.

---

# 12. Position PID with Sine Target

The target can be generated inside the control loop:

````ino

float timeSeconds =
    micros() / 1000000.0;

float targetPosition =
    sineCenter +
    sineAmplitude *
    sin(
      2.0 * PI *
      sineFrequency *
      timeSeconds
    );
````

Then the PID compares:

````text

Target sine wave
       |
       v
     PID
       |
       v
    Motor
       |
       v
   Encoder
       |
       v
Actual position
````

The goal is for the actual position to follow the target as closely as possible.

---

# 13. Moving from Position Control to Speed Control

Position control asks:

"Where should the motor be?"

Speed control asks:

"How fast should the motor be rotating?"

For speed control, we need to calculate velocity from the encoder.

The basic equation is:

````text

speed = change in position / change in time
````

In code:

````ino

deltaPosition =
    currentPosition - previousPosition;

speed =
    deltaPosition / dt;
````


This gives encoder counts per second.

---

# 14. Measuring Speed

Example:

````ino

long currentPosition;
long previousPosition = 0;

unsigned long currentTime;
unsigned long previousTime = 0;

float speed;

void loop() {

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currentPosition = encoderPosition;
  }

  currentTime = micros();

  float dt =
      (currentTime - previousTime) / 1000000.0;

  long deltaPosition =
      currentPosition - previousPosition;

  speed =
      deltaPosition / dt;

  previousPosition = currentPosition;
  previousTime = currentTime;

  Serial.println(speed);

  delay(10);
}
````

If the encoder counts are increasing:

````text

speed > 0
````

If the encoder counts are decreasing:

````text

speed < 0
````

Therefore, speed automatically contains direction.

---

# 15. Converting Encoder Speed to RPM

If the encoder provides a known number of counts per revolution:

````ino

float countsPerRevolution = 600;
````

then:

````text

revolutions/second =
counts/second / countsPerRevolution
````

RPM is:

````text

RPM =
(counts/second / countsPerRevolution) × 60
````

In code:

````ino

float rpm =
    (speed / countsPerRevolution) * 60.0;
````

For example:

````text

600 counts/sec
600 counts/revolution

= 1 revolution/sec

= 60 RPM
````

---


# 16. Speed PID

Now instead of comparing position, compare desired speed to measured speed.

````text

speedError =
targetSpeed - measuredSpeed
````

For example:

````text

Target speed = 100 RPM
Actual speed = 80 RPM

Error = 20 RPM
````

The PID increases motor power.

If:

````text
Target speed = 100 RPM
Actual speed = 120 RPM

Error = -20 RPM
````

The PID decreases motor power.

---

# 17. Speed PID Structure

The control loop becomes:

````text

Target Speed
      |
      v
   Speed Error
      |
      v
     PID
      |
      v
 PWM / Direction
      |
      v
    Motor
      |
      v
   Encoder
      |
      v
Measured Speed
      |
      +----------> PID
````

This is different from position control.

Position control:

````text

Target Position → Position PID → Motor
````

Speed control:

````text

Target Speed → Speed PID → Motor
````

# 18. Basic Speed PID Code

````ino

#include <util/atomic.h>

#define ENCA 2
#define ENCB 3

#define PWM_PIN 5
#define IN1 9
#define IN2 10

volatile long encoderPosition = 0;

// Encoder
long previousPosition = 0;

// Timing
unsigned long previousTime;

// Speed
float measuredSpeed = 0;

// Target speed in encoder counts/second
float targetSpeed = 300;

// PID
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.0;

float integral = 0;
float previousError = 0;

void setup() {

  Serial.begin(115200);

  // Encoder
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENCA),
    readEncoder,
    RISING
  );

  // Motor
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  setMotor(0);

  previousTime = micros();
}

void loop() {

  // -----------------------------------------------
  // Read encoder
  // -----------------------------------------------

  long currentPosition;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currentPosition = encoderPosition;
  }


  // -----------------------------------------------
  // Calculate dt
  // -----------------------------------------------

  unsigned long currentTime = micros();

  float dt =
      (currentTime - previousTime) / 1000000.0;

  previousTime = currentTime;

  if (dt <= 0) {
    return;
  }


  // -----------------------------------------------
  // Calculate speed
  // -----------------------------------------------

  long deltaPosition =
      currentPosition - previousPosition;

  measuredSpeed =
      deltaPosition / dt;

  previousPosition =
      currentPosition;


  // -----------------------------------------------
  // Speed PID
  // -----------------------------------------------

  float error =
      targetSpeed - measuredSpeed;

  integral += error * dt;

  float derivative =
      (error - previousError) / dt;

  float output =
      Kp * error +
      Ki * integral +
      Kd * derivative;

  previousError =
      error;


  // -----------------------------------------------
  // Limit output
  // -----------------------------------------------

  output =
      constrain(output, -255, 255);


  // -----------------------------------------------
  // Motor
  // -----------------------------------------------

  setMotor(output);


  // -----------------------------------------------
  // Serial Plotter
  // -----------------------------------------------

  Serial.print("Target:");
  Serial.print(targetSpeed);

  Serial.print("\tActual:");
  Serial.println(measuredSpeed);


  delay(10);
}

void readEncoder() {

  int b =
      digitalRead(ENCB);

  if (b == HIGH) {
    encoderPosition++;
  }
  else {
    encoderPosition--;
  }
}

void setMotor(float command) {

  if (command > 0) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    analogWrite(
      PWM_PIN,
      (int)command
    );
  }

  else if (command < 0) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(
      PWM_PIN,
      (int)(-command)
    );
  }

  else {

    analogWrite(PWM_PIN, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
````

---

# 19. Position PID vs Speed PID

The two controllers have different purposes.


| Feature | Position PID | Speed PID |
|---|---|---:|
| Feedback | Encoder position | Encoder speed |
| Target | Position | Speed |
| Example | 1000 counts | 300 counts/sec |
| Output | Motor PWM | Motor PWM |
| Main use | Move to a position | Maintain velocity |


Position:

````text

"I want the wheel here."
````

Speed:

````text

"I want the wheel moving this fast."
````

---

# 20. Recommended Development Sequence

Do not try to tune everything at once.

Follow this sequence:

## Step 1 — Encoder

First verify that:

````text

Rotate forward → counts increase
Rotate backward → counts decrease
````

Do this with the motor disconnected if necessary.

---

## Step 2 — Motor

Verify:

````text

Positive command → motor moves forward
Negative command → motor moves backward
Zero command → motor stops
````

Test the L298N without PID first.

---

## Step 3 — Position Measurement

Turn the wheel manually and verify that the encoder position changes correctly.

Determine:

````text

counts per revolution
````

if needed.

---

## Step 4 — Position PID

Start with a fixed target:

````ino

long targetPosition = 1000;
````

Tune:

````text

Kp
Kd
Ki
````

in that order.

---

# Step 5 — Sine Position Tracking

Once fixed-position control works, replace the fixed target with:

````ino

targetPosition =
    sineCenter +
    sineAmplitude *
    sin(2.0 * PI * sineFrequency * time);
````

Plot:

````text

Target
Actual
````

and compare the two curves.

---

# Step 6 — Measure Speed

Calculate:

````text

counts/second
````

from encoder position changes.

Then verify that:

````text

Forward → positive speed
Reverse → negative speed
````

---

## Step 7 — Speed PID

Use:

````text

Target Speed
      ↓
    PID
      ↓
    Motor
      ↓
   Encoder
      ↓
Measured Speed
````

Tune the speed controller independently from the position controller.

---

# 21. Important Considerations

## Encoder Resolution

Know exactly how your encoder's specifications define its counts/revolution.

Some encoders specify pulses per revolution based on one channel, while others specify counts after decoding both channels.

The code above uses:

````text

RISING edge of channel A
````


so it is a relatively low-resolution decoding method.

If higher resolution is needed, the encoder can be decoded using more edges:

````text

A rising
A falling
B rising
B falling
````

This is commonly called x4 decoding.

---

## Motor Deadband

DC motors often do not move at very low PWM values.

For example:

````text

PWM = 20  → motor doesn't move
PWM = 40  → motor doesn't move
PWM = 60  → motor starts moving
````

This is especially important with an L298N.

A PID controller may calculate a small output that is technically nonzero but isn't enough to overcome friction.

Deadband compensation can be added later.

---

## Integral Windup

The integral term can continue accumulating while the motor is saturated at:

````text

+255
````

or:

````text

-255
````

This can cause overshoot and slow recovery.

For a more advanced controller, add integral limits or anti-windup.

For example:

````ino

integral = constrain(integral, -1000, 1000);
````

---


## Control Loop Frequency

The PID controller needs a consistent update rate.

For example:

````ino

delay(10);
````

gives approximately:


````text

100 Hz
````

in ideal conditions.

For better control, it is usually preferable to use a timer or elapsed-time check rather than relying on `delay()`.


---


# 22. Final Control Architecture

Once everything is working, the complete system can be thought of as:

````text

                    +----------------------+
                    |      Target          |
                    | Position / Speed     |
                    +----------+-----------+
                               |
                               v
                         +-----------+
                         |    PID    |
                         +-----+-----+
                               |
                               v
                         Motor Command
                               |
                               v
                         +-----------+
                         |   L298N   |
                         +-----+-----+
                               |
                               v
                         +-----------+
                         | DC Motor  |
                         +-----+-----+
                               |
                               v
                         +-----------+
                         | Encoder   |
                         +-----+-----+
                               |
                               v
                       Position / Speed
                               |
                               +----------+
                                          |
                                          v
                                       PID
````

The fundamental progression is:

````text

Encoder
   ↓
Position
   ↓
Position PID
   ↓
Speed calculation
   ↓
Speed PID
````

Once speed control works reliably, the system can be extended into more advanced control structures such as:

````text

Position Controller
        ↓
   Desired Speed
        ↓
    Speed PID
        ↓
      Motor
        ↓
     Encoder
````

This is a common architecture for robotic wheels because the outer position controller can command a desired speed, while the inner speed controller handles the motor's actual velocity.





