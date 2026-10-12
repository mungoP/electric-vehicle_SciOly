#ifndef MOTOR_H
#define MOTOR_H

// DRV8833: left motor on IN1/IN2 (OUT1/OUT2), right motor on IN3/IN4 (OUT3/OUT4)
void motor_init(void);

// signed duty -1..1 per motor, positive = forward (after the invert flags)
void motor_set(float left, float right);

// both inputs high: motor terminals shorted, wheels resist turning
void motor_brake(void);

// both inputs low: wheels spin freely
void motor_coast(void);

#endif
