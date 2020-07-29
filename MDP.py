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

class MarkovDecisionProcess:

    """A Markov Decision Process, defined by an states, actions, transition model and reward function."""

    def __init__(self, map, gamma=.9):
        self.map = map
        self.reward = [0 for index in range(len(map))]
        for curState in map:
            for (nextState, length, action, reward) in curState:
                self.reward[nextState] = reward
        self.gamma = gamma
        self.states = []
        for index in range(len(map)):
            self.states.append(index)

    def R(self, state):
        """return reward for this state."""
        return self.reward[state]

    def actions(self, state):
        """return set of actions that can be performed in this state"""
        list_actions = []
        for (nextState, length, action, reward) in self.map[state]:
            list_actions.append(action)
        return list_actions

    def T(self, state, action):
        """for a state and an action, return (probability, result-state) pairs."""
        for (nextState, length, act, reward) in self.map[state]:
            if (act == action):
                return [(0.8, nextState)]
        return [(0,0)]

    def lengthState(self):
        return len(self.map)
