
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

```text
Forward:

A: __|‾‾|__|‾‾|__
B: ____|‾‾|__|‾‾

Reverse:

B: __|‾‾|__|‾‾|__
A: ____|‾‾|__|‾‾
```

The encoder interrupt is triggered by channel A.

## Basic Encoder Code

```arduino
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
```
