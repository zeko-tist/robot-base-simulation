import numpy as np

from isaacsim.core.utils.types import ArticulationActions

from .constants import (
    LEFT_WHEEL,
    RIGHT_WHEEL,
    DEFAULT_SPEED,
    TURN_SPEED,
)


class ZekoController:

    def __init__(self, robot):

        self.robot = robot

        self.left_idx = robot.dof_names.index(LEFT_WHEEL)
        self.right_idx = robot.dof_names.index(RIGHT_WHEEL)

    def drive(self, left_speed, right_speed):

        vel = np.zeros((1, self.robot.num_dof))

        vel[0, self.left_idx] = left_speed
        vel[0, self.right_idx] = right_speed

        action = ArticulationActions(
            joint_velocities=vel
        )

        self.robot.apply_action(action)

    def forward(self):
        self.drive(DEFAULT_SPEED, DEFAULT_SPEED)

    def backward(self):
        self.drive(-DEFAULT_SPEED, -DEFAULT_SPEED)

    def left(self):
        self.drive(-TURN_SPEED, TURN_SPEED)

    def right(self):
        self.drive(TURN_SPEED, -TURN_SPEED)

    def stop(self):
        self.drive(0.0, 0.0)
