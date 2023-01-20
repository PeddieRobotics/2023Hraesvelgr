// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double JOYSTICK_THRESHHOLE = 0.05; //placeholder value 
  public static final double INCHES_TO_METERS = 0.0254; // Converting from inches to meters
  public static final double TRACKWIDTH_M = 0.56; //test value .6 
  public static final double WHEELBASE_M = 0.56; //test value .575
  public static final double WHEELDIAMETER_IN = 3.90; //This is in inches 
  public static final double WHEELRADIUS_M = (WHEELDIAMETER_IN / 2) * INCHES_TO_METERS; // converting wheel radius to meters 
  public static final double METERS_PER_SEC_TO_RPM = 30 / (Math.PI* WHEELRADIUS_M); //Converting Meters per Second to Rotation Per Minute 
  public static final double DRIVE_GEAR_RATIO = 6.75; //Gear ratio for the standard wheel (motor to wheel rotation: 8:16:1)
  public static final double ANGLE_GEAR_RATIO = 21.428571428571427;

  public static class DriveConstants {

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.WHEELBASE_M / 2.0, Constants.TRACKWIDTH_M / 2.0),
            new Translation2d(Constants.WHEELBASE_M / 2.0, -Constants.TRACKWIDTH_M / 2.0),
            new Translation2d(-Constants.WHEELBASE_M / 2.0, Constants.TRACKWIDTH_M / 2.0),
            new Translation2d(-Constants.WHEELBASE_M / 2.0, -Constants.TRACKWIDTH_M / 2.0));

    public static final int kTranslation = 30;
    public static final int kRotation = 25;
    public static final double kFrontLeftDriveP = 0.15;
    public static final double kFrontLeftAngleP = 0.2;
    public static final double kFrontRightDriveP = 0.15;
    public static final double kFrontRightAngleP = 0.2;
    public static final double kBackLeftDriveP = 0.15;
    public static final double kBackLeftAngleP = 0.2;
    public static final double kBackRightDriveP = 0.15;
    public static final double kBackRightAngleP = 0.2;
    public static final double kDeadband = 0.10;

    public static final double kMaxSpeedMetersPerSecond = 2.5;
    public static final double kMaxAcceleration = 3.00;
    public static final double kMaxAngularSpeed = Math.PI/3;
    public static final double kMaxAngularAcceleration = Math.PI / 2;

    public static final double kTranslationSlew = 4.5;
    public static final double kRotationSlew = 10.0;

    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
            * Math.pow(DriveConstants.kDeadband, 3);
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
            * Math.pow(DriveConstants.kDeadband, 3);

    public final static int kFrontLeftTurningEncoderChannel = 3;
    public final static int kFrontRightTurningEncoderChannel = 2;
    public final static int kBackLeftTurningEncoderChannel = 1;
    public final static int kBackRightTurningEncoderChannel = 0;

    public final static double kFrontLeftAngularOffset = -3.207 + 0.053 - 0.029 + 0.05; // 4
    public final static double kFrontRightAngularOffset = -1.909 + 0.06 - 0.084 - 0.06; // 3
    public final static double kBackLeftAngularOffset = -2.65 + 0.066 - 0.05 - 0.062; // 2
    public final static double kBackRightAngularOffset = -2.94 - 0.059 + 0.073 + 0.058; // 1

    public final static double[] kKeepAnglePID = { 0.350, 0, 0 };

    public static final double BEAM_BALANCED_DRIVE_kP = 0; //starting value for p
    public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
    public static final double BEAM_BALANCED_ANGLE_THRESHOLD_DEGREES = -1;
}

public static class GlobalConstants {
    public static final double kVoltCompensation = 12.6;
}

public static final class AutoConstants {
    public static final double kMaxAcceleration = 2.50;
    public static final double kMaxSpeed = 3.25; // Maximum Sustainable Drivetrain Speed under Normal Conditions &
                                                 // Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI/2; // Maximum Angular Speed desired. NOTE: Robot can exceed
                                                           // this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed
                                                           // this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kPXController = 3.0;
    public static final double kPYController = 3.0;
    public static final double kPTranslationController = 3.0;
    public static final double kPThetaController = 3.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeed, kMaxAngularAccel); // Creates a trapezoidal motion for the auto rotational commands
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
/*
* Convert wheel radius from inches to meters: 2 * 0.0254 =
* 0.0508
* 
* Convert target vel in m/s to rev/min (wheel): 3.704 * 30 / (pi * 0.0508) =
* 696.272
* 
* Convert rev/min wheel to rev/min motor using gear ratio: 696.272 * 8.16 =
* 5681.576 (YAY!!!!!)
*/