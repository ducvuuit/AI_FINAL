#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from Control import Control

class BFS:
    def __init__(self, control, mapInp):
        self.control = control
        self.map = mapInp
        self.listDirect = {
            'L': -90,
            'U': 0,
            'R': 90,
            'D': -180
        }

    def run(self, startState, endState):
        mark = []
        state = []
        state.append((startState, 0, [startState]))
        print(len(self.map))
        for index in range(len(self.map)):
            mark.append(True)
        mark[startState] = False
        while (len(state) > 0):
            (currentState, currentLength, currentPath) = state[0]
            del state[0]
            for (curState, length, action) in self.map[currentState]:
                if (mark[curState]):
                    mark[curState] = False
                    routes = currentPath + [curState]
                    if (curState == endState):
                        return routes
                    state.append((curState, currentLength, routes))
        return ''

    def play(self, path, startDirect):
        linesensor = ColorSensor(Port.S3)
        curDirect = startDirect
        desDirect = ''
        for index in range(len(path) - 1):
            goState = path[index]
            desState = path[index + 1]
            for (state, length, direct, reward) in self.map[goState]:
                if (state == desState):
                    desDirect = direct
                    break
            angle = (self.listDirect[desDirect] - self.listDirect[curDirect])
            if (angle < -180):
                angle += 90
            if (angle > 180):
                angle -= 90
            #Run robot
            print(goState, desState, angle)
            if (angle == -90):
                self.control.rotate_left(linesensor, drive_speed = 50)
            if (angle == 90):
                self.control.rotate_right(linesensor, drive_speed = 50)
            if (angle == 180):
                self.control.rotate_back(linesensor, drive_speed = 50)

            self.control.run(linesensor,drive_speed = 20)
            #
            curDirect = desDirect
        return curDirect

