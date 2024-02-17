// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

        public static class GlobalConstants {
                public static final boolean kUseLEDLights = true;
        }

        public static class OIConstants {
                public static final boolean kUseDebugModeLayout = true;
                public static final boolean kUsePreScorePose = true;
                public static final boolean kReturnL3ConeInvertedToPreScore = false;
                public static final boolean kReturnForwardL2L3ScoringPosesToPreScore = true;

                public static final double kDrivingDeadband = 0.1;
        }

        public static class BlinkinConstants {
                public static final int kPwmPort = 1;
        }

        public static class DriveConstants {
                // Chassis Configuration
                // Distance between the centers of left and right wheels on the robot
                public static final double kTrackWidth = Units.inchesToMeters(20.5);
                // Distance between the front and back wheels on the robot
                public static final double kWheelBase = Units.inchesToMeters(24.5);

                public static final double kBaseRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2;

                public static final double kRealMaxSpeedMetersPerSecond = 4.459224;
                public static final double kMaxFloorSpeed = 1.0 * kRealMaxSpeedMetersPerSecond;
                public static final double kMaxAcceleration = 3;
                public static final double kMaxAngularSpeed = 4 * Math.PI / 3;
                public static final double kMaxAngularAcceleration = 2 * Math.PI / 3;

                public static final double kNormalModeTranslationSpeedScale = 1.0;
                public static final double kNormalModeRotationSpeedScale = 1.0;
                public static final double kSlowModeTranslationSpeedScale = 0.4;
                public static final double kSlowModeRotationSpeedScale = 0.4;
                public static final double kCardinalDirectionSpeedScale = 0.3;

                public static final Translation2d[] swerveModuleLocations = {
                                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
                };

                public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                                swerveModuleLocations[0],
                                swerveModuleLocations[1],
                                swerveModuleLocations[2],
                                swerveModuleLocations[3]);

                // Angular offsets of the modules relative to the chassis in radians
                public static final double kFrontLeftChassisAngularOffset = 3 * Math.PI / 2;
                public static final double kFrontRightChassisAngularOffset = 0;
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kBackRightChassisAngularOffset = Math.PI / 2;

                // Translation and Rotation Slew Rates
                public static final boolean kUseRateLimit = true;
                public static final double kDirectionSlewRate = 4.5; // 3; // radians per second
                public static final double kMagnitudeSlewRate = 4.5; // 3; // percent per second (1 = 100%)
                public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

                public final static int kFrontLeftTurningEncoderChannel = 3;
                public final static int kFrontRightTurningEncoderChannel = 2;
                public final static int kBackLeftTurningEncoderChannel = 1;
                public final static int kBackRightTurningEncoderChannel = 0;

                // "Snap to angle" algorithm parameters
                public final static double[] kSnapToAnglePID = { 0.350, 0, 0 };

                // "Correct heading" algorithm parameters
                public static final double kHeadingCorrectionP = 0.05;
                public static final double kHeadingCorrectionTolerance = 2.0;
        }

        public static final class ModuleConstants {
                // The MAXSwerve module can be configured with one of three pinion gears: 12T,
                // 13T, or 14T.
                // This changes the drive speed of the module (a pinion gear with more teeth
                // will result in a robot that drives faster).
                public static final int kDrivingMotorPinionTeeth = 13;

                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of
                // the steering motor in the MAXSwerve Module.
                public static final boolean kTurningEncoderInverted = true;

                // Calculations required for driving motor conversion factors and feed forward
                public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = 0.0762; // correction
                public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
                // teeth on the bevel pinion
                public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                                * kWheelCircumferenceMeters) / kDrivingMotorReduction;

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
                public static final double kPTranslationController = 3.5; //3.5 old 
                public static final double kITranslationController = 0.0;
                public static final double kDTranslationController = 0.0;

                public static final double kPThetaController = 1.0; // 1.5 new
                public static final double kIThetaController = 0.0;
                public static final double kDThetaController = 0.4; // 0.4 for new autos

                // Charge station balance algorithm parameters
                // Tune based on field.
                public static final double kCSDebounceTime = 0.1;
                public static final double kOnCSDegree = 9.0;
                public static final double kCSLevelDegree = 11.5;
                public static final double kCSClimbDistance = 0.75;
                public static final double kCSOverrunMaxApproachDist = 2.0;
                public static final double kCSOverrunRemainingClimbDist = 0.5;
                
        }

        public static final class NeoMotorConstants {
                public static final double kFreeSpeedRpm = 5676;
        }

        public static final class LimelightConstants {
                public static final double kLimelightHeight = 28.0; // inches... UPDATED
                public static final double kLimelightPanningAngle = 0;
                public static final double kLimelightAngle = 0;

                public static final double kLimelightHeadingBound = 1.0;
                public static final double kLimeLightTranslationScoringAngleBound = 0.3;
                public static final double kLimeLightTranslationSingleSSAngleBound = 1.0;
                
                // public static final Translation2d[] columnDestinationCoords = {new Translation2d(),new Translation2d(1.02743,.512826),new Translation2d(1.02743,1.071626),
                //                                 new Translation2d(1.02743,1.630426),new Translation2d(1.02743,2.189226),new Translation2d(1.02743,2.748026),
                //                                 new Translation2d(1.02743,3.306826),new Translation2d(1.02743,3.865626),new Translation2d(1.02743,4.424426),
                //                                 new Translation2d(1.02743,4.983226)};
                // public static final double robotOffsetToGoal = .37+.4+1;

                public static final double kDriveScaleScoreAlign = 1.0;
                public static final double kDriveScaleSingleSSAlign = 1.0;

                //apriltag align, turn 
                public static final double sourceTurnP = 0.05; //.04
                public static final double sourceTurnI = 0.0002; 
                public static final double sourceTurnD = 0.0; 
                public static final double sourceTurnThresh1 = 7.0; 
                public static final double sourceTurnThresh2 = 1.5; 
                public static final double sourceTurnFF = 0.0; 
                public static final double sourceTurnIZone = 5; 

                //apriltag align, move 
                public static final double sourceMoveP = 0.058; //.08
                public static final double sourceMoveI = 0.0025;
                public static final double sourceMoveD = 0.005; 
                public static final double sourceMoveThresh = 1.5; //0.5
                public static final double sourceMoveFF = 0.03; //.01
                public static final double sourceMoveIZone = 5; 

                public static final double sourceMaxThrottle = .6; 

                //megatag botpose constants 
                public static final double bpMoveP = 1.2; 
                public static final double bpMoveI = 0.0001; 
                public static final double bpMoveD = 0.0; 
                public static final double bpMoveFF = 0.05; 
                public static final double bpMoveThresh = 0.09; 

                public static final double bpTurnP = 0.02; 
                public static final double bpTurnI = 0.00002; 
                public static final double bpTurnD = 0.0; 
                public static final double bpTurnThresh = 1; 
                public static final double bpTurnFF = 0.0; 

        }

        public static final class ShoulderConstants {
                // Do not change the below numbers without consultation, extremely dangerous!
                public static final int kMaxCurrent = 60;

                // Position PID contants
                public static final double kPositionP = 0.013;
                public static final double kPositionI = 0.000001;
                public static final double kPositionD = 0.00003;
                public static final double kPositionIz = 7;

                // Shoulder feedforward
                public static final double kGVolts = 0.55;
                public static final double kVVoltSecondPerRad = 0.0;
                public static final double kAVoltSecondSquaredPerRad = 0.0;

                // Smart Motion controller parameters (velocity PID constants)
                public static final double kP = 0.0000005;
                public static final double kI = 0.0000005;
                public static final double kD = 0;
                public static final double kIz = 1500;
                public static final double kFF = 0.000162;

                public static final double kSmartMotionSlowSetpointTol = 1.0;
                public static final double kSmartMotionSlowMinVel = 0.0; // rpm
                public static final double kSmartMotionSlowMaxVel = 1000.0; // rpm
                public static final double kSmartMotionSlowMaxAccel = 5000.0; // rpm / sec

                public static final double kSmartMotionRegularSetpointTol = 1.0;
                public static final double kSmartMotionRegularMinVel = 0.0; // rpm
                public static final double kSmartMotionRegularMaxVel = 5500.0; // rpm
                public static final double kSmartMotionRegularMaxAccel = 5000.0; // rpm / sec
                
                // ONLY FOR INVERSION FROM STATIONARY POSE.  Do not use to "uninvert" or while robot is moving....
                // Be warned...
                public static final double kSmartMotionFastSetpointTol = 1.0;
                public static final double kSmartMotionFastMinVel = 0.0; // rpm
                public static final double kSmartMotionFastMaxVel = 5500.0; // rpm
                public static final double kSmartMotionFastMaxAccel = 30000.0; // rpm / sec

                // Soft limits
                public static final double kAngleMin = -75;
                public static final double kAngleMax = 155;

                // Angles (poses) start here
                public static final double kHomeAngle = -75.0;
                public static final double kTransitoryAngle = -40.0;
                public static final double kStowedAngle = -75.0;
                public static final double kPreScoreAngle = 0.0;

                public static final double kL1Angle = -75.0;

                // Shoulder is not fully extended out
                // Currently unused
                public static final double kCompactFloorConeAngle = -62;
                public static final double kCompactFloorCubeAngle = -60;

                // Shoulder is fully extended out
                public static final double kExtendedFloorConeAngle = -26.5;
                public static final double kExtendedFloorCubeAngle = -36;

                public static final double kL2ConeAngle = 16.0;
                public static final double kL2CubeAngle = 16.0;

                public static final double kL3CubeForwardAngle = 8.5;
                public static final double kL3CubeInvertedAngle = 155.0;
                public static final double kL3ConeForwardAngle = 11.0;
                public static final double kL3ConeInvertedAngle = 155.0;

                public static final double kDoubleSSConeAngle = 27.5;
                public static final double kSingleSSConeAngle = -75.0;
                public static final double kSingleSSCubeAngle = -75.0;

                // Used generically in the code for checking if we are "close enough" to a pose
                // Not currently the same as the Smart Motion tolerance used onboard the PID
                // controller.
                public static final double kSetpointTolerance = 1.0; // degrees

                public static final double kMotorReduction = 192.0; // 192:1 shoulder reduction
                public static final double kEncoderConversionFactor = 360.0/kMotorReduction;

                public static final double kFloorIntakeConeMaxVelToStow = 1200;
                public static final double kFloorIntakeConeMaxAccelToStow = 2500;

                public static final double kFloorIntakeCubeMaxVelToStow = 1500;
                public static final double kFloorIntakeCubeMaxAccelToStow = 5000;

                public static final double kScoringPoseMaxVelToStow = 1650;
                public static final double kScoringPoseMaxAccelToStow = 7000;

        }

        public static final class WristConstants {
                // Do not change the below numbers without consultation, extremely dangerous!
                public static final int kMaxCurrent = 20;

                // Position PID constants
                public static final double kPositionP = 0.015;
                public static final double kPositionI = 0.000005;
                public static final double kPositionD = 0.0001;
                public static final double kPositionIz = 4.0;
                public static final double kPositionFF = 0.0;
            
                // Wrist feedforward
                public static final double kGVolts = 0.0;
                public static final double kVVoltSecondPerRad = 0.0;
                public static final double kAVoltSecondSquaredPerRad = 0.0;

                // Smart Motion controller parameters (velocity PID constants)
                public static final double kP = 0.00007;
                public static final double kI = 0.000000005;
                public static final double kD = 0;
                public static final double kIz = 0;
                public static final double kFF = 0.0001;

                public static final double kSmartMotionSlowSetpointTol = 1.0;
                public static final double kSmartMotionSlowMinVel = 0.0; // rpm
                public static final double kSmartMotionSlowMaxVel = 1000.0; // rpm
                public static final double kSmartMotionSlowMaxAccel = 1000.0; // rpm / sec

                public static final double kSmartMotionRegularSetpointTol = 1.0;
                public static final double kSmartMotionRegularMinVel = 0.0; // rpm
                public static final double kSmartMotionRegularMaxVel = 2000.0; // rpm
                public static final double kSmartMotionRegularMaxAccel = 2000.0; // rpm / sec

                public static final double kSmartMotionFastSetpointTol = 1.0;
                public static final double kSmartMotionFastMinVel = 0.0; // rpm
                public static final double kSmartMotionFastMaxVel = 3000.0; // rpm
                public static final double kSmartMotionFastMaxAccel = 3000.0; // rpm / sec
        
                // Angles (poses) start here
                public static final double kHomeAngle = 5.0;
                public static final double kStowedAngle = 28.0;
                public static final double kPreScoreAngle = 28.0;

                public static final double kL1Angle = 133.0;

                // Shoulder is not fully extended out
                // Currently unused
                public static final double kCompactFloorConeAngle = 174.0;
                public static final double kCompactFloorCubeAngle = 183.0;

                // Shoulder is fully extended out
                public static final double kExtendedFloorConeAngle = 149.0;
                public static final double kExtendedFloorCubeAngle = 138.0;

                public static final double kL2ConeAngle = 191.0;
                public static final double kL2CubeAngle = 186.0;

                public static final double kL3CubeForwardAngle = 112.0;
                public static final double kL3CubeInvertedAngle = 118.0;
                public static final double kL3ConeForwardAngle = 109;
                public static final double kL3ConeInvertedAngle = 108.0;

                public static final double kDoubleSSConeAngle = 192.0;
                public static final double kSingleSSConeAngle = 78.0;
                public static final double kSingleSSCubeAngle = 40.0;

                public static final double kTransitoryAngle = 38.0;
                public static final double kMonitorConeAlignmentAngle = 65.0;
                public static final double kMonitorCubeAlignmentAngle = 60.0;

                public static final double kSetpointTolerance = 1.0; // degrees

                public static final double kMotorReduction = 100.571429; // old was 113.777777777777:1
                public static final double kEncoderConversionFactor = 360.0 / kMotorReduction;
                public static final double kAbsoluteEncoderConversionFactor = 360.0/1.571429; /// old was 360.0/1.77777777777777;

                public static final double kAbsoluteEncoderZeroOffset = 172.24;

                // Soft limits
                public static final float kAngleMin = 4;
                public static final float kAngleMax = 215;

        }

        public static final class ClawConstants {
                public static final int kClawMotorCurrentLimit = 35;

                public static final double kConeIntakeSpeed = -1;
                public static final double kCubeIntakeSpeed = -1;
                public static final double kCubeSingleSSIntakeSpeed = -0.3;
                public static final double kConeSingleSSIntakeSpeed = -1;

                public static final double kConeL1OuttakeSpeed = 0.5;
                public static final double kConeL2OuttakeSpeed = 1;
                public static final double kConeL3ForwardOuttakeSpeed = 1;
                public static final double kConeL3InvertedOuttakeSpeed = 1;

                public static final double kCubeL1OuttakeSpeed = 0.3;
                public static final double kCubeL2OuttakeSpeed = 0.5;
                public static final double kCubeL3ForwardOuttakeSpeed = 0.8;
                public static final double kCubeL3InvertedOuttakeSpeed = 0.3;

                public static final double kCubeHoldSpeed = -0.05;

                public static final double kMaximumGamepieceMonitorTime = 0.5;

                public static final double kOperatorEjectSpeed = 0.1;
                public static final double kOperatorFastEjectSpeed = 1.0;

                public static final double kOperatorIntakeSpeed = -0.1;
                public static final double kOperatorFastIntakeSpeed = -1.0;
        }
}