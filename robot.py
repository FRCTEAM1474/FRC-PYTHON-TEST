
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

WHEELBASE_M = 0.577342
TRACKWIDTH_M = 0.577342

# Original YAGSL mechanical values
DRIVE_GEAR_RATIO = 8.14
STEER_GEAR_RATIO = 12.8
WHEEL_DIAMETER_M = 0.1016

# These mechanical values are retained for
# future closed-loop velocity control.

# SPARK MAX settings
DRIVE_CURRENT_LIMIT = 40
STEER_CURRENT_LIMIT = 20
RAMP_RATE = 0.25
NOMINAL_VOLTAGE = 12.0

# Drive speed and power settings
MAX_SPEED_MPS = 4.4196
MAX_TRANSLATION = 1.0
MAX_ROTATION = 1.5

DRIVE_POWER_LIMIT = 1.0
STEER_POWER_LIMIT = 0.5

# Pigeon 2
PIGEON_ID = 14
FIELD_RELATIVE = True


# ==========================================
# SWERVE MODULE CONFIGURATION
# ==========================================

# Offsets are provided in degrees.
# Verify these with the actual wheel positions.
# Also verify FR steer CAN ID 13.

MODULE_CONFIG = {
    "FL": {
        "drive": 6,
        "steer": 2,
        "encoder": 10,
        "offset_deg": 82.146,
        "drive_sign": 1,
        "steer_sign": 1,
        "encoder_sign": -1,
    },
    "FR": {
        "drive": 5,
        "steer": 13,
        "encoder": 9,
        "offset_deg": 337.351,
        "drive_sign": 1,
        "steer_sign": 1,
        "encoder_sign": -1,
    },
    "BL": {
        "drive": 7,
        "steer": 3,
        "encoder": 11,
        "offset_deg": 101.50,
        "drive_sign": 1,
        "steer_sign": 1,
        "encoder_sign": -1,
    },
    "BR": {
        "drive": 8,
        "steer": 4,
        "encoder": 12,
        "offset_deg": 82.966,
        "drive_sign": 1,
        "steer_sign": 1,
        "encoder_sign": -1,
    },
}


# ==========================================
# HELPER FUNCTIONS
# ==========================================

def clamp(value, limit):
    return max(-limit, min(limit, value))


def deadband(value):
    if abs(value) < 0.10:
        return 0.0
    return value


# ==========================================
# SWERVE MODULE CLASS
# ==========================================

class SwerveModule:

    def __init__(self, name, config):
        self.name = name

        brushless = rev.SparkLowLevel.MotorType.kBrushless

        # Create SPARK MAX motor controllers
        self.drive_motor = rev.SparkMax(
            config["drive"],
            brushless
        )

        self.steer_motor = rev.SparkMax(
            config["steer"],
            brushless
        )

        # ==================================
        # DRIVE MOTOR CONFIGURATION
        # ==================================

        drive_settings = rev.SparkMaxConfig()

        drive_settings.smartCurrentLimit(
            DRIVE_CURRENT_LIMIT
        )

        drive_settings.openLoopRampRate(
            RAMP_RATE
        )

        drive_settings.voltageCompensation(
            NOMINAL_VOLTAGE
        )

        # Encoder conversion settings removed
        # to avoid unsupported firmware parameters.

        # ==================================
        # STEERING MOTOR CONFIGURATION
        # ==================================

        steer_settings = rev.SparkMaxConfig()

        steer_settings.smartCurrentLimit(
            STEER_CURRENT_LIMIT
        )

        steer_settings.openLoopRampRate(
            RAMP_RATE
        )

        steer_settings.voltageCompensation(
            NOMINAL_VOLTAGE
        )

        # ==================================
        # APPLY CONFIGURATION
        # ==================================

        self.drive_motor.configure(
            drive_settings,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters
        )

        self.steer_motor.configure(
            steer_settings,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters
        )

        # ==================================
        # CANCODER CONFIGURATION
        # ==================================

        self.encoder = CANcoder(
            config["encoder"]
        )

        self.offset_rot = (
            config["offset_deg"] / 360.0
        )

        self.drive_sign = config["drive_sign"]
        self.steer_sign = config["steer_sign"]
        self.encoder_sign = config["encoder_sign"]

        # ==================================
        # STEERING PID
        # ==================================

        self.steer_pid = PIDController(
            0.6, 0.0, 0.0
        )

        self.steer_pid.enableContinuousInput(
            -math.pi,
            math.pi
        )

    # ======================================
    # GET CURRENT MODULE ANGLE
    # ======================================

    def get_angle(self):
        signal = self.encoder.get_absolute_position()

        if not signal.status.is_ok():
            return None

        rotations = (
            signal.value - self.offset_rot
        ) * self.encoder_sign

        return Rotation2d.fromRotations(rotations)

    # ======================================
    # CONTROL SWERVE MODULE
    # ======================================

    def set_desired_state(self, desired_state):

        current_angle = self.get_angle()

        if current_angle is None:
            self.stop()
            return False

        # Optimize steering direction
        desired_state.optimize(current_angle)

        # Reduce speed while wheels are misaligned
        desired_state.cosineScale(current_angle)

        # Steering PID calculation
        steer_output = self.steer_pid.calculate(
            current_angle.radians(),
            desired_state.angle.radians()
        )

        steer_output = clamp(
            steer_output,
            STEER_POWER_LIMIT
        )

        # Open-loop drive output
        drive_output = clamp(
            desired_state.speed / MAX_SPEED_MPS,
            DRIVE_POWER_LIMIT
        )

        # Apply motor outputs
        self.steer_motor.set(
            steer_output * self.steer_sign
        )

        self.drive_motor.set(
            drive_output * self.drive_sign
        )

        # ==================================
        # SMARTDASHBOARD DIAGNOSTICS
        # ==================================

        wpilib.SmartDashboard.putNumber(
            f"{self.name} Drive Output",
            drive_output * self.drive_sign
        )

        wpilib.SmartDashboard.putNumber(
            f"{self.name} Angle Deg",
            current_angle.degrees()
        )

        return True

    # ======================================
    # STOP MODULE MOTORS
    # ======================================

    def stop(self):
        self.drive_motor.stopMotor()
        self.steer_motor.stopMotor()


# ==========================================
# MAIN ROBOT CLASS
# ==========================================

class MyRobot(wpilib.TimedRobot):

    def reset_gyro_for_alliance(self):
        alliance = wpilib.DriverStation.getAlliance()

        if alliance == wpilib.DriverStation.Alliance.kRed:
            target_yaw = 0.0

        elif alliance == wpilib.DriverStation.Alliance.kBlue:
            target_yaw = 180.0

        else:
            print("Alliance unknown! Gyro not reset.")
            return

        result = self.gyro.set_yaw(target_yaw)

        if result.is_ok():
            print(f"Gyro reset to {target_yaw} degrees")
        else:
            print(f"Gyro reset failed: {result}")

    def robotInit(self):

        self.controller = wpilib.XboxController(0)

        self.gyro = Pigeon2(PIGEON_ID)

        # Initialize four swerve modules
        self.modules = {
            name: SwerveModule(name, config)
            for name, config in MODULE_CONFIG.items()
        }

        half_length = WHEELBASE_M / 2.0
        half_width = TRACKWIDTH_M / 2.0

        # Kinematics order: FL, FR, BL, BR
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(
                half_length, half_width
            ),
            Translation2d(
                half_length, -half_width
            ),
            Translation2d(
                -half_length, half_width
            ),
            Translation2d(
                -half_length, -half_width
            ),
        )

        print("Swerve initialized!")

    # ======================================
    # TELEOPERATED CONTROL
    # ======================================

    def teleopPeriodic(self):
        # Reset heading with Y button
        if self.controller.getYButtonPressed():
            self.stop_all()
            self.reset_gyro_for_alliance()
            return
        vx = -deadband(
            self.controller.getLeftY()
        ) * MAX_TRANSLATION
        vx = -deadband(
            self.controller.getLeftY()
        ) * MAX_TRANSLATION

        vy = -deadband(
            self.controller.getLeftX()
        ) * MAX_TRANSLATION

        omega = -deadband(
            self.controller.getRightX()
        ) * MAX_ROTATION

        # Stop when controller sticks are neutral
        if vx == 0 and vy == 0 and omega == 0:
            self.stop_all()
            return

        # Field-relative driving
        if FIELD_RELATIVE:

            yaw = self.gyro.get_yaw()

            if not yaw.status.is_ok():
                self.stop_all()
                return

            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx,
                vy,
                omega,
                Rotation2d.fromDegrees(yaw.value)
            )

        # Robot-relative driving
        else:
            speeds = ChassisSpeeds(
                vx,
                vy,
                omega
            )

        # Calculate individual module states
        states = self.kinematics.toSwerveModuleStates(
            speeds
        )

        # Limit requested wheel speeds
        states = (
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
                states,
                MAX_SPEED_MPS
            )
        )

        # Command all four modules
        for name, state in zip(
            ("FL", "FR", "BL", "BR"),
            states
        ):
            self.modules[name].set_desired_state(
                state
            )

    # ======================================
    # STOP ALL MODULES
    # ======================================

    def stop_all(self):
        for module in self.modules.values():
            module.stop()

    def disabledInit(self):
        self.stop_all()
