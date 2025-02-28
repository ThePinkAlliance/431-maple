// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.Gains;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.09);
        public static final double kDriveMotorGearRatio = 1 / 8.14;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.47;

        public static final Gains kBackLeftSteerGains = new Gains(.34, 0.0, 0);
        public static final Gains kBackRightSteerGains = new Gains(.34, 0.0, 0);
        public static final Gains kFrontRightSteerGains = new Gains(.34, 0.0, 0);
        public static final Gains kFrontLeftSteerGains = new Gains(.34, 0.0, 0);
    }

    public static final class OIConstants {
        public static final double kJoystickDeadband = 0.05;
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(23.75);

        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(23.75);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 8;
        public static final int kBackRightDriveMotorPort = 2;

        public static final int kFrontLeftTurningMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 4;
        public static final int kFrontRightTurningMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 1;

        /** Port numbers for all the cancoders. */
        public static final int kFrontLeftDriveCANCoderPort = 12;

        public static final int kBackLeftDriveCANCoderPort = 6;
        public static final int kFrontRightDriveCANCoderPort = 9;
        public static final int kBackRightDriveCANCoderPort = 3;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        /** These values where determined by lining up all the wheels and recording the outputed positions. */
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.552;

        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -2.207;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.446;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.0184;

        // This is the max speed without load.
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.437;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 1; // 0.96
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
                kPhysicalMaxAngularSpeedRadiansPerSecond / 2.8;
        public static double kTeleDriveSpeedReduction = 1;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2.5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.5;
    }

    public static final class CoralSubsystemConstants {
        public static final int kElevatorMotorCanId = 4;
        public static final int kArmMotorCanId = 3;
        public static final int kIntakeMotorCanId = 2;

        public static final class ElevatorSetpoints {
            public static final int kFeederStation = 0;
            public static final int kLevel1 = 0;
            public static final int kLevel2 = 0;
            public static final int kLevel3 = 100;
            public static final int kLevel4 = 150;
        }

        public static final class ArmSetpoints {
            public static final double kFeederStation = 33;
            public static final double kLevel1 = 0;
            public static final double kLevel2 = 2;
            public static final double kLevel3 = 2;
            public static final double kLevel4 = 19;
        }

        public static final class IntakeSetpoints {
            public static final double kForward = 0.5;
            public static final double kReverse = -0.5;
        }
    }

    public static final class AlgaeSubsystemConstants {
        public static final int kIntakeMotorCanId = 13;
        public static final int kPivotMotorCanId = 14;

        public static final class ArmSetpoints {
            public static final double kStow = 18.5;
            public static final double kHold = 11.5;
            public static final double kDown = 0;
        }

        public static final class IntakeSetpoints {
            public static final double kForward = 0.5;
            public static final double kReverse = -0.5;
            public static final double kHold = 0.25;
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class SimulationRobotConstants {
        public static final double kPixelsPerMeter = 20;

        public static final double kElevatorGearing = 25; // 25:1
        public static final double kCarriageMass = 4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
        public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
        public static final double kMinElevatorHeightMeters = 0.922; // m
        public static final double kMaxElevatorHeightMeters = 1.62; // m

        public static final double kArmReduction = 60; // 60:1
        public static final double kArmLength = 0.433; // m
        public static final double kArmMass = 4.3; // Kg
        public static final double kMinAngleRads = Units.degreesToRadians(-50.1); // -50.1 deg from horiz
        public static final double kMaxAngleRads = Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

        public static final double kIntakeReduction = 135; // 135:1
        public static final double kIntakeLength = 0.4032262; // m
        public static final double kIntakeMass = 5.8738; // Kg
        public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
        public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
        public static final double kIntakeShortBarLength = 0.1524;
        public static final double kIntakeLongBarLength = 0.3048;
        public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
    }
}
