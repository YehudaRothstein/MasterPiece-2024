from hub import motion_sensor, port
import math, motor, motor_pair, time

perror = 0
I = 0
r = 2.7
yaw = 0.0
kp = 0.15
ki = 0.00
kd = 0.8
tp = 0
ti = 0
td = 0

motor_pair.pair(motor_pair.PAIR_1, port.C, port.D)
'''
motor.run(port.A, 400)
motor.run(port.B,400)
'''
motor.reset_relative_position(port.C, 0)
motor.reset_relative_position(port.D, 0)
motion_sensor.reset_yaw(0)


def PID(kp, ki, kd, Error):
    global perror
    global I
    P = Error
    I = Error - I
    D = Error - perror
    perror = P
    PID = P * kp + I * ki + D * kd
    return PID


def drive(cm: int, velocity: int, angle: int):
    motion_sensor.reset_yaw(0)
    motor.reset_relative_position(port.C, 0)
    motor.reset_relative_position(port.D, 0)
    if cm > 0:
        while (((motor.relative_position(port.C) * -1 + motor.relative_position(port.D)) / 360 * r * math.pi) < cm):
            v = int(PID(kp, ki, kd, motion_sensor.tilt_angles()[0] - angle))
            motor_pair.move_tank(motor_pair.PAIR_1, velocity + v, velocity - v)
            print(motion_sensor.tilt_angles()[0])
        motor_pair.stop(motor_pair.PAIR_1)
    else:
        while (((motor.relative_position(port.C) * -1 + motor.relative_position(port.D)) / 360 * r * math.pi) > cm):
            v = int(PID(tp, ti, td, motion_sensor.tilt_angles()[0] - angle))
            motor_pair.move_tank(motor_pair.PAIR_1, -(velocity - v), -(velocity + v))
            print(motion_sensor.tilt_angles()[0])
        motor_pair.stop(motor_pair.PAIR_1)


def reset_yaw():
    motion_sensor.reset_yaw(0)


def turn(angle):
    global yaw
    reset_yaw()
    angle = angle * 10

    # שמאל
    while yaw < angle - 1 or yaw > angle + 1:
        motor_pair.move_tank(motor_pair.PAIR_1, int((angle - yaw) / 2.1 + 60) * 1, int(((angle - yaw) / 2.1 + 60)))
        yaw = motion_sensor.tilt_angles()[0]
    motor_pair.stop(motor_pair.PAIR_1)
    return


def move_arm(selected_motor: str, degrees):
    if selected_motor == 'a':
        motor.run_for_degrees(port.A, degrees, 500)
    if selected_motor == 'b':
        motor.run_for_degrees(port.B, degrees, 500)


drive(3456789023456, 700, 0)
move_arm('a', 2000)
move_arm('b', -150)

