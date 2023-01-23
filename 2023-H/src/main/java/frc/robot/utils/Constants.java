// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

        public static class GlobalConstants {
                public static final double kVoltCompensation = 12.6;
        }

        public static class MeasurementConstants {
                // Chassis Configuration
                // Distance between the centers of left and right wheels on the robot
                public static final double TRACKWIDTH_M = Units.inchesToMeters(24);
                // Distance between the front and back wheels on the robot
                public static final double WHEELBASE_M = Units.inchesToMeters(28);
                public static final double WHEELDIAMETER_IN = 3.0;
                public static final double WHEELRADIUS_M = Units.inchesToMeters(WHEELDIAMETER_IN / 2);
                public static final double METERS_PER_SEC_TO_RPM = 30 / (Math.PI * WHEELRADIUS_M);
                public static final double DRIVE_GEAR_RATIO = 6.75;
                public static final double ANGLE_GEAR_RATIO = 21.428571428571427;
        }

        public static class OIConstants {
                public static final double JOYSTICK_THRESHHOLE = 0.05;
                public static final double kDrivingDeadband = 0.05;
        }

        public static class DriveConstants {
                public static final double kMaxSpeedMetersPerSecond = 3.5;
                public static final double kMaxAcceleration = 3.00;
                public static final double kMaxAngularSpeed = 2 * Math.PI;
                public static final double kMaxAngularAcceleration = 2 * Math.PI / 2;

                public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                                new Translation2d(MeasurementConstants.WHEELBASE_M / 2.0,
                                                MeasurementConstants.TRACKWIDTH_M / 2.0),
                                new Translation2d(MeasurementConstants.WHEELBASE_M / 2.0,
                                                -MeasurementConstants.TRACKWIDTH_M / 2.0),
                                new Translation2d(-MeasurementConstants.WHEELBASE_M / 2.0,
                                                MeasurementConstants.TRACKWIDTH_M / 2.0),
                                new Translation2d(-MeasurementConstants.WHEELBASE_M / 2.0,
                                                -MeasurementConstants.TRACKWIDTH_M / 2.0));

                // Angular offsets of the modules relative to the chassis in radians
                public static final double kFrontLeftChassisAngularOffset = 3 * Math.PI / 2;
                public static final double kFrontRightChassisAngularOffset = 0;
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kBackRightChassisAngularOffset = Math.PI / 2;

                // Translation and Rotation Slew Rates
                public static final double kTranslationSlewRate = 4.5; // meters per second
                public static final double kRotationSlewRate = 4.5; // radians per second

                public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
                                * Math.pow(OIConstants.kDrivingDeadband, 3);
                public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
                                * Math.pow(OIConstants.kDrivingDeadband, 3);

                public final static int kFrontLeftTurningEncoderChannel = 3;
                public final static int kFrontRightTurningEncoderChannel = 2;
                public final static int kBackLeftTurningEncoderChannel = 1;
                public final static int kBackRightTurningEncoderChannel = 0;

                public final static double[] kSnapToAnglePID = { 0.350, 0, 0 };

                public static final double BEAM_BALANCED_DRIVE_kP = 0; // starting value for p
                public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
                public static final double BEAM_BALANCED_ANGLE_THRESHOLD_DEGREES = -1;
        }

        public static final class ModuleConstants {
                // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
                // This changes the drive speed of the module (a pinion gear with more teeth will result in a robot that drives faster).
                public static final int kDrivingMotorPinionTeeth = 12;

                // Invert the turning encoder, since the output shaft rotates in the opposite direction of
                // the steering motor in the MAXSwerve Module.
                public static final boolean kTurningEncoderInverted = true;

                 // Calculations required for driving motor conversion factors and feed forward
                public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = 0.0762;
                public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
                public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

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

                public static final int kDrivingMotorCurrentLimit = 50; // amps
                public static final int kTurningMotorCurrentLimit = 20; // amps
        }

        public static final class AutoConstants {
                public static final double kMaxAcceleration = 2.50;
                public static final double kMaxSpeed = 3.25;
                public static final double kMaxAngularSpeed = Math.PI / 2;
                public static final double kMaxAngularAccel = Math.PI;
                public static final double kPXController = 3.0;
                public static final double kPYController = 3.0;
                public static final double kPTranslationController = 3.0;
                public static final double kPThetaController = 3.0;

                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeed, kMaxAngularAccel); 
        }

        public static final class NeoMotorConstants {
                public static final double kFreeSpeedRpm = 5676;
        }

        public static final class LimelightConstants {
                public static final double kLimelightP = 0.075;
                public static final double kLimelightI = 0.0;
                public static final double kLimelightD = 0.0;
                public static final double kLimelightFF = 0.15;
                public static final double kLimeLightAngleBound = 0.5;
                public static final double kTurningMultiplier = 5.0;
        }
}