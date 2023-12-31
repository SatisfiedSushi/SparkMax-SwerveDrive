// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        public static final double kTrackingP = 0.02; // 0.02
        public static final double kTrackingI = 0.0; // 0.003
        public static final double kTrackingD = 0.0; // 0.001
        public static final double kTrackingTolerance = 0.01;
        public static final double kTrackingToleranceDerivative = 6;
        public static final double kTrackingIntergratorRangeMin = -0.7;
        public static final double kTrackingIntergratorRangeMax = 0.7;

        public static final double kAvoidingP = 0.02;
        public static final double kAvoidingI = 0;
        public static final double kAvoidingD = 0;
        public static final double kAvoidingTolerance = 0.01;
        public static final double kAvoidingToleranceDerivative = 6;
        public static final double kAvoidingIntergratorRangeMin = -0.7;
        public static final double kAvoidingIntergratorRangeMax = 0.7;

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(18);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(18);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 4; // neo
        public static final int kRearLeftDrivingCanId = 2; // neo
        public static final int kFrontRightDrivingCanId = 6; // neo
        public static final int kRearRightDrivingCanId = 8; // neo

        public static final int kFrontLeftTurningCanId = 3; // neo 550
        public static final int kRearLeftTurningCanId = 1; // neo 550
        public static final int kFrontRightTurningCanId = 5; // neo 550
        public static final int kRearRightTurningCanId = 7; // neo 550

        public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 30; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.1;
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

    public static final class Neo550MotorConstants {
        public static final double kFreeSpeedRpm = 11000;
    }

    public static final class LimelightConstants {
        public static final double kLLHeight = Units.inchesToMeters(24.5);
        public static final double kLLPitch = Units.degreesToRadians(0);
        public static final double kMinObjectAvoidanceDistance = Units.inchesToMeters(12);
        public static final double kObjectHeight = Units.inchesToMeters(12);
        public static final double kObjectPitch = Units.degreesToRadians(0);

        public static final int[] kBlueAprilTags = new int[]{1,2,3,4};
        public static final int[] kRedAprilTags = new int[]{5,6,7,8};
    }

    public static final class BalanceConstants {
        public static final class CenterTrackingPID {
            public static final double kP = 0.02;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kTolerance = 0.01;
            public static final double kToleranceDerivative = 6;
            public static final double kIntergratorRangeMin = -0.7;
            public static final double kIntergratorRangeMax = 0.7;
        }

        public static final class ChargePadPID {
            public static final double kP = 0.02;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kTolerance = 0.01;
            public static final double kToleranceDerivative = 6;
            public static final double kIntergratorRangeMin = -0.7;
            public static final double kIntergratorRangeMax = 0.7;
        }
    }

    public static final class TurretConstants {
        public static final int kShootMotorCanID = 10; // neo
        public static final int kRotateMotorCanID = 11; // neo 550
        public static final int kIntakeMotorCanID = 12; // neo 550

        public static final int kShootMotorCurrentLimit = 30; // amps
        public static final int kRotateMotorCurrentLimit = 20; // amps
        public static final int kIntakeMotorCurrentLimit = 20; // amps

        public static final IdleMode kShootMotorIdleMode = IdleMode.kCoast;
        public static final IdleMode kRotateMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kIntakeMotorIdleMode = IdleMode.kBrake;

        public static final double turnP = 0;
        public static final double turnI = 0;
        public static final double turnD = 0;

        // 8.3521:1 (2 stage UltraPlanetary Cartridge 3:1) -> 12:70 (spur to spur)
        public static final double kRotateMotorReduction = 52.5;
        public static final double kTurretDiameterMeters = Units.inchesToMeters(7.2);
        public static final double kTurretCircumferenceMeters = Units.inchesToMeters(kTurretDiameterMeters * Math.PI);
        public static final double kTurretMotorFreeSpeedRps = (Neo550MotorConstants.kFreeSpeedRpm
                / kTurretCircumferenceMeters)
                / kRotateMotorReduction;

        public static final double kRotateEncoderPositionFactor = (kTurretCircumferenceMeters / kRotateMotorReduction); // meters
        public static final double kRotateEncoderVelocityFactor = (kTurretCircumferenceMeters / kRotateMotorReduction)
                / 60.0; // meters per second
    }

    public static final class PPvar {
        public static final double squareSize = Units.inchesToMeters(48);
        public static final double figureEightSize = Units.inchesToMeters(24);
        public static final double starSize = Units.inchesToMeters(24);
        public static final double maxVel = 0.00001;
        public static final double maxAccel = 0.0000001;
    }

    public static final boolean kIsTeamBlue = true;
}
