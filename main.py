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
from MDP import MarkovDecisionProcess

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
#ev3.speaker.beep()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
'''
map = [
    [(1,20,'U', 0)],
    [(0,20,'D',0), (2,50,'U',0), (3,15,'L',0)],
    [(1,50,'D',0), (4,15, 'L',1)],
    [(1,15,'R',0), (4,50,'U',1)],
    [(3,50,'D',0), (2,15,'R',0)]
] 
'''
map = [
[(1,20,'U', 0)],
[(0,20,'D',0), (2,50,'L',0), (4,15,'U',0)],
[(1,15,'R',0),(3,15,'L',0),(5,15,'U',0)],
[(2,50,'L',0),(6,15,'U',0)],
[(1,15,'D',0),(5,50,'L',0),(8,15,'U',-1)],
[(2,50,'D',0),(4,15,'R',0)],
[(3,15,'D',0),(7,15,'R',1)],
[(6,15,'L',0),(8,20,'R',-1),(11,50,'U',0)],
[(7,20,'L',1),(4,15,'D',0),(10,15,'U',0)],
[(10,15,'L',0)],
[(8,15,'D',-1),(9,15,'R',0),(11,15,'L',0)],
[(10,15,'R',0),(12,15,'L',10),(7,15,'D',1)],
[(11,15,'R',0)]
]
#line_sensor = ColorSensor(Port.S3)
linefollow = Control(left_motor,right_motor,threshold=21, proportion=-1.2)
# #list_direction = linefollow.run(line_sensor, drive_speed=20)
# #linefollow.rotate_left(line_sensor, drive_speed=20)
bfs = BFS(control = linefollow, mapInp = map)
# path = bfs.run(0, 4)
# print(path)
# bfs.play(path, 'U')

mdp = MarkovDecisionProcess(map, gamma = 0.9)

def value_iteration():
    """
    Solving the MDP by value iteration.
    returns utility values for states after convergence
    """
    states = mdp.states
    actions = mdp.actions
    T = mdp.T
    R = mdp.R
    gamma = mdp.gamma
    eps = 1e-9
    #initialize value of all the states to 0 (this is k=0 case)
    V1 = {s: 0 for s in states}
    while True:
        V = V1.copy()
        delta = 0
        for s in states:
            #Bellman update, update the utility values
            #probability = 0.8
            max_value = 1e-5
            for a in actions(s):
                for (p, s1) in T(s,a):
                    max_value = max(max_value, p*V[s1])
            V1[s] = R(s) + gamma * max_value
            #calculate maximum difference in value
            delta = max(delta, abs(V1[s] - V[s]))

        #check for convergence, if values converged then return V
        if delta < eps * (1 - gamma) / gamma:
            return V

def best_policy(V):
    """
    Given an MDP and a utility values V, determine the best policy as a mapping from state to action.
    returns policies which is dictionary of the form {state1: action1, state2: action2}
    """
    states = mdp.states
    actions = mdp.actions
    pi = {}
    for s in states:
        pi[s] = max(actions(s), key=lambda a: expected_utility(a, s, V))
    return pi


def expected_utility(a, s, V):
    """returns the expected utility of doing a in state s, according to the MDP and V."""
    T = mdp.T
    return sum([p * V[s1] for (p, s1) in mdp.T(s, a)])

def render(pi,startState, endState):
    T = mdp.T
    curState = startState
    direct = 'U'
    while (curState != endState):
        action = pi[curState]
        next = T(curState, action)
        (probability, nextState) = next[0]
        # probability, nextState = next[0]
        path = [curState, nextState]
        direct = bfs.play(path, startDirect=direct)
        curState = nextState



V = value_iteration()
pi = best_policy(V)
print(pi)
render(pi, startState = 0, endState = 12)

