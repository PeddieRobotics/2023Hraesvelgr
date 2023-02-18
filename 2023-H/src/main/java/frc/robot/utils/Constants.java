// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

        public static class GlobalConstants {
                public static final double kVoltCompensation = 12.6;
        }

        public static class OIConstants {
                public static final double kDrivingDeadband = 0.1;
        }

        public static class BlinkinConstants {
                public static final int kPwmPort = 1; // DETERMINE REAL PWM PORT FOR BLINKIN CONTROLLER!!! 1 is an arbitrary filler value
        }

        public static class DriveConstants {
                // Chassis Configuration
                // Distance between the centers of left and right wheels on the robot
                public static final double kTrackwidth = Units.inchesToMeters(20.5);
                // Distance between the front and back wheels on the robot
                public static final double kWheelbase = Units.inchesToMeters(24.5);

                public static final double kRealMaxSpeedMetersPerSecond = 4.117848;
                public static final double kMaxFloorSpeed = 0.75 * kRealMaxSpeedMetersPerSecond;
                public static final double kMaxAcceleration = 3;
                public static final double kMaxAngularSpeed = 2*Math.PI;
                public static final double kMaxAngularAcceleration = 2*Math.PI / 3;

                public static final double kNormalModeTranslationSpeedScale = 1.0;
                public static final double kNormalModeRotationSpeedScale = 1.0;
                public static final double kSlowModeTranslationSpeedScale = 0.25;
                public static final double kSlowModeRotationSpeedScale = 0.55;
                public static final double kCardinalDirectionSpeedScale = 0.3;

                public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                                new Translation2d(kWheelbase / 2.0,
                                                kTrackwidth / 2.0),
                                new Translation2d(kWheelbase / 2.0,
                                                -kTrackwidth / 2.0),
                                new Translation2d(-kWheelbase / 2.0,
                                                kTrackwidth / 2.0),
                                new Translation2d(-kWheelbase/ 2.0,
                                                -kTrackwidth / 2.0));

                // Angular offsets of the modules relative to the chassis in radians
                public static final double kFrontLeftChassisAngularOffset = 3*Math.PI/2;
                public static final double kFrontRightChassisAngularOffset = 0;
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kBackRightChassisAngularOffset = Math.PI/2;

                // Translation and Rotation Slew Rates
                public static final double kTranslationSlewRate = 4; // meters per second
                public static final double kRotationSlewRate = 4; // radians per second

                public final static int kFrontLeftTurningEncoderChannel = 3;
                public final static int kFrontRightTurningEncoderChannel = 2;
                public final static int kBackLeftTurningEncoderChannel = 1;
                public final static int kBackRightTurningEncoderChannel = 0;

                // "Snap to angle" algorithm parameters
                public final static double[] kSnapToAnglePID = { 0.350, 0, 0 };

                // "Beam balance" algorithm parameters
                public static final double kPBeamBalanceDrive = 0; // starting value for p
                public static final double kBeamBalanceGoalDegrees = 0;
                public static final double kBeamBalanceAngleThresholdDegrees = -1;

                // "Correct heading" algorithm parameters
                public static final double kHeadingCorrectionP = 0.05;
                public static final double kHeadingCorrectionTolerance = 2.0;
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

                public static final double kTurningP = 0.5;
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
                public static final double kLimelightHeight = 22.8346; // inches
                public static final double kLimelightPanningAngle = 0;
                public static final double kLimelightAngle = 0;
        }

        public static final class ShoulderConstants{
                 // Do not change the below numbers without consultation, extremely dangerous!
                public static final int kMaxCurrent = 30; // 40;
            
                public static final double kP = 0.013;
                public static final double kI = 0.000002;    
                public static final double kD = 0.00003;
                public static final double kIz = 7;
                public static final double kFF = 0.0;
                public static final double kAngleMin = -75;
                public static final double kAngleMax = 155;
            
                public static final double kShoulderStowedAngle = -75.0;
                public static final double kShoulderLLSeekAngle = -75.0;
                public static final double kShoulderLevelOneAngle = -75.0;

                public static final double kShoulderFloorConeAngle = -59.0;
                public static final double kShoulderFloorCubeAngle = -59.0;

                public static final double kShoulderLevelTwoConeAngle = 5.0;
                public static final double kShoulderLevelTwoCubeAngle = 5.0;

                public static final double kShoulderLevelThreeTransition = 90.0;
                public static final double kShoulderLevelThreeCubeLobAngle = 10.0;
                public static final double kShoulderLevelThreeCubeDunkAngle = 157.0;
                public static final double kShoulderLevelThreeConeAngle = 157.0;
                public static final double kShoulderHumanPlayerConeAngle = 20.0;
                public static final double kShoulderHumanPlayerSingleSS = -75.0;

                public static final double kShoulderSetpointTolerance = 1.0; // degrees

                public static final double kSVolts = 0.0;
                public static final double kGVolts = 0.3;
                public static final double kVVoltSecondPerRad = 5.77;
                public static final double kAVoltSecondSquaredPerRad = 0.05;

                public static final double kShoulderMotorReduction = 296; // 10368:35 or approximately 296:1;

                public static final double kShoulderEncoderConversionFactor = 225.0/185.0; // angular delta in degrees divided by encoder delta in native rotations
        }

        public static final class WristConstants{
                // Do not change the below numbers without consultation, extremely dangerous!
                public static final int kMaxCurrent = 25;
            
                public static final double kP = 0.005;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kIz = 0.0;
                public static final double kFF = 0.0;
                public static final double kAngleMin = -75;
                public static final double kAngleMax = 135;
            
                public static final double kSVolts = 0.0;
                public static final double kGVolts = 0.0;
                public static final double kVVoltSecondPerRad = 0.0;
                public static final double kAVoltSecondSquaredPerRad = 0.0;
            
                public static final double kWristStowedAngle = 70.0;
                public static final double kWristLLSeekAngle = -10.0;
                public static final double kWristLevelOneAngle = -10.0;

                public static final double kWristFloorConeAngle = -62.0;
                public static final double kWristFloorCubeAngle = -75.0;
                
                public static final double kWristLevelTwoConeAngle = -87.4;
                public static final double kWristLevelTwoCubeAngle = -87.4;

                public static final double kWristLevelThreeCubeTransition = 103.0;
                public static final double kWristLevelThreeCubeLobAngle = -29.4;
                public static final double kWristLevelThreeCubeDunkAngle = 2.5;
                public static final double kWristLevelThreeConeAngle = 2.5;
                public static final double kWristHumanPlayerConeAngle = -80.0;
                public static final double kWristHumanPlayerSingleSS = 30.0;

                public static final double kWristSetpointTolerance = 1.0; // degrees

                public static final double kWristMotorReduction =  160; // 160:1
                public static final double kWristEncoderConversionFactor =  103/41.761; // angular delta in degrees divided by encoder delta in native rotations
            
        }
        
        public static final class ClawConstants {
                public static final int kClawMotorCurrentLimit = 40;

                // Below intake/outtake speeds need fixing (made up placeholders)
                public static final double kConeIntakeSpeed = -1;
                public static final double kConeOuttakeSpeed = 1;
                public static final double kCubeIntakeSpeed = -1;
                public static final double kCubeOuttakeSpeed = 1;
        }
}