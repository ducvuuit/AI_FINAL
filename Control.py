#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class Control:
    def __init__(self, left_motor, right_motor,  threshold, proportion, wheel_diameter=55.5, axle_track=104):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track
        self.threshold = threshold
        self.proportion = proportion
        self.robot = DriveBase(left_motor,right_motor,wheel_diameter, axle_track)
    
    def run(self, line_sensor, drive_speed):
        self.robot.straight(20)
        directions = []
        another_sensor = ColorSensor(Port.S1)
        while (True):
            current = line_sensor.reflection()
            deviation = current - self.threshold
            turn_rate = deviation * self.proportion
            self.robot.drive(drive_speed, turn_rate)
            if (current < 11):
                directions.append('R')
                if (another_sensor.reflection() < 20):
                    directions.append('L')
                break
        self.robot.stop()
        return directions

    def rotate_right(self, line_sensor, drive_speed):
        self.robot.straight(70)
        wait(10)
        while (True):
            current = line_sensor.reflection()
            if (current < 20):
                break
            self.robot.drive(drive_speed, 70)
        self.robot.stop()
        while (True):
            current = line_sensor.reflection()
            if (current > 20):
                break
            self.robot.drive(drive_speed, 70)
        self.robot.stop()

    def rotate_left(self, line_sensor, drive_speed):
        self.robot.straight(30)
        wait(10)
        another_sensor = ColorSensor(Port.S1)
        while (True):
            current = another_sensor.reflection()
            if (current < 15):
                break
            self.robot.drive(drive_speed, -70)
        self.robot.stop()
        while (True):
            current = line_sensor.reflection()
            if (current < 20):
                break
            self.robot.drive(drive_speed, -70)
        self.robot.stop()

