package frc.robot.utils;

public class RobotMap{

    // Drive Motors
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 20;
    public static final int kFrontRightDrivingCanId = 30;
    public static final int kRearRightDrivingCanId = 40;

    // Turning Motors
    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 21;
    public static final int kFrontRightTurningCanId = 31;
    public static final int kRearRightTurningCanId = 41;
    //Sensors 
    //TODO: Currently arbitrary values, update later
    public static final int kWristLimitSensor = 0;
    //public static final int kShoulderLimitSensor = 1;
    public static final int kClawFrontSensor = 2;
    public static final int kClawBackSensor = 3;
    
    // Arm Motors
    public static final int kShoulderMotorMaster = 50;
    public static final int kShoulderMotorFollower = 51;
    public static final int kWristMotor = 61;

    public static final int kClawMotor = 60;
}
