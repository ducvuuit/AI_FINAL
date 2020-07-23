#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from Control import Control
from BFS import BFS
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
#ev3.speaker.beep()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

map = [
    [(1,20,'U')],
    [(0,20,'D'), (2,50,'U'), (3,15,'L')],
    
    [(1,50,'D'), (4,15, 'L')],
    [(1,15,'R'), (4,50,'U')],
    [(3,50,'D'), (2,15,'R')]
]

#line_sensor = ColorSensor(Port.S3)
linefollow = Control(left_motor,right_motor,threshold=21, proportion=-1.2)
#list_direction = linefollow.run(line_sensor, drive_speed=20)
#linefollow.rotate_left(line_sensor, drive_speed=20)
bfs = BFS(control = linefollow, mapInp = map)
path = bfs.run(0, 4)
print(path)
bfs.play(path, 'U')