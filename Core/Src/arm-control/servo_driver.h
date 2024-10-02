#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

void initialize_servos(void);
void move_servo_to_position(int servo_id, int angle);

#endif
