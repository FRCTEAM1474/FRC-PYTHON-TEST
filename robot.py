
import math

import wpilib
import rev

from phoenix6.hardware import CANcoder, Pigeon2
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
)


# ==========================================
# CHASSIS CONFIGURATION
# ==========================================

# Measure between module steering-axis centers, in meters.
WHEELBASE_M = 0.577342
TRACKWIDTH_M = 0.577342


PIGEON_ID = 14

# Replace None with your real CAN IDs and offsets.
# offset_rot = raw CANcoder reading when the wheel
# is pointing straight forward (in rotations).

MODULE_CONFIG = {
    "FL": {
        "drive": 6, "steer": 2,
        "encoder": 10, "offset_rot": 82.146,
        "drive_sign": 1, "steer_sign": 1,
        "encoder_sign": 1,
    },
    "FR": {
        "drive": 5, "steer": 1,
        "encoder": 9, "offset_rot": 337.351,
        "drive_sign": 1, "steer_sign": 1,
        "encoder_sign": 1,
    },
    "BL": {
        "drive": 7, "steer": 3,
        "encoder": 11, "offset_rot": 101.50,
        "drive_sign": 1, "steer_sign": 1,
        "encoder_sign": 1,
    },
    "BR": {
        "drive": 8, "steer": 4,
        "encoder": 12, "offset_rot": 82.966,
        "drive_sign": 1, "steer_sign": 1,
        "encoder_sign": 1,
    },
}

# Conservative initial output limits.
DRIVE_POWER_LIMIT = 0.12
STEER_POWER_LIMIT = 0.12

# These are command scaling values for initial
# open-loop testing, NOT measured robot speeds.
MAX_TRANSLATION = 0.6
MAX_ROTATION = 1.0
REFERENCE_WHEEL_SPEED = 1.0

# Leave disabled until gyro direction is verified.
FIELD_RELATIVE = False


def clamp(value, limit):
    return max(-limit, min(limit, value))


def deadband(value):
    return 0.0 if abs(value) < 0.10 else value


class SwerveModule:

    def __init__(self, name, config):
        self.name = name
        brushless = rev.SparkLowLevel.MotorType.kBrushless

        self.drive_motor = rev.SparkMax(
            config["drive"], brushless
        )

        self.steer_motor = rev.SparkMax(
            config["steer"], brushless
        )

        self.encoder = CANcoder(config["encoder"])

        self.offset = config["offset_rot"]
        self.drive_sign = config["drive_sign"]
        self.steer_sign = config["steer_sign"]
        self.encoder_sign = config["encoder_sign"]

        self.steer_pid = PIDController(0.6, 0, 0)
        self.steer_pid.enableContinuousInput(
            -math.pi, math.pi
        )

    def get_angle(self):
        signal = self.encoder.get_absolute_position()

        if not signal.status.is_ok():
            return None

        rotations = (
            signal.value - self.offset
        ) * self.encoder_sign

        return Rotation2d.fromRotations(rotations)

    def set_desired_state(self, desired_state):
        current_angle = self.get_angle()

        # Never move a module with invalid angle data.
        if current_angle is None:
            self.stop()
            return False

        # Select shortest steering path.
        desired_state.optimize(current_angle)
        desired_state.cosineScale(current_angle)

        turn_output = self.steer_pid.calculate(
            current_angle.radians(),
            desired_state.angle.radians()
        )

        turn_output = clamp(
            turn_output, STEER_POWER_LIMIT
        )

        drive_output = (
            desired_state.speed / REFERENCE_WHEEL_SPEED
        ) * DRIVE_POWER_LIMIT

        drive_output = clamp(
            drive_output, DRIVE_POWER_LIMIT
        )

        self.steer_motor.set(
            turn_output * self.steer_sign
        )

        self.drive_motor.set(
            drive_output * self.drive_sign
        )

        return True

    def stop(self):
        self.drive_motor.stopMotor()
        self.steer_motor.stopMotor()


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        # Prevent running with placeholder configuration.
        if (
            WHEELBASE_M is None
            or TRACKWIDTH_M is None
            or PIGEON_ID is None
        ):
            raise ValueError(
                "Enter chassis dimensions and Pigeon ID"
            )

        for name, config in MODULE_CONFIG.items():
            required = (
                "drive", "steer", "encoder", "offset_rot"
            )
            if any(config[key] is None for key in required):
                raise ValueError(
                    f"Missing configuration for {name}"
                )

        self.controller = wpilib.XboxController(0)
        self.gyro = Pigeon2(PIGEON_ID)

        self.modules = {
            name: SwerveModule(name, config)
            for name, config in MODULE_CONFIG.items()
        }

        half_length = WHEELBASE_M / 2
        half_width = TRACKWIDTH_M / 2

        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(half_length, half_width),
            Translation2d(half_length, -half_width),
            Translation2d(-half_length, half_width),
            Translation2d(-half_length, -half_width),
        )

        print("Swerve initialized!")

    def teleopPeriodic(self):
        vx = -deadband(
            self.controller.getLeftY()
        ) * MAX_TRANSLATION

        vy = -deadband(
            self.controller.getLeftX()
        ) * MAX_TRANSLATION

        omega = -deadband(
            self.controller.getRightX()
        ) * MAX_ROTATION

        # Stop motors when sticks are neutral.
        if vx == 0 and vy == 0 and omega == 0:
            self.stop_all()
            return

        if FIELD_RELATIVE:
            yaw = self.gyro.get_yaw()

            if not yaw.status.is_ok():
                self.stop_all()
                return

            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, omega,
                Rotation2d.fromDegrees(yaw.value)
            )
        else:
            speeds = ChassisSpeeds(vx, vy, omega)

        states = self.kinematics.toSwerveModuleStates(
            speeds
        )

        states = (
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
                states, REFERENCE_WHEEL_SPEED
            )
        )

        for name, state in zip(
            ("FL", "FR", "BL", "BR"), states
        ):
            self.modules[name].set_desired_state(state)

    def stop_all(self):
        for module in self.modules.values():
            module.stop()

    def disabledInit(self):
        self.stop_all()
