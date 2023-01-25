package frc.robot.subsystems;

import javax.sql.rowset.spi.TransactionalWriter;

import com.ctre.phoenix.GadgeteerUartClient.GadgeteerProxyType;
// import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
// import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMapH;
import frc.robot.utils.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain drivetrain;

  // Swerve Modules
  private final MAXSwerveModule[] swerveModules;
  private final MAXSwerveModule frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule;

  // Gyroscope Sensor
  private final ADIS16470_IMU gyro;
  private double heading;

  // Swerve Drive 
  private SwerveModuleState[] swerveModuleStates;
  private SwerveModulePosition[] swerveModulePositions;

  private final SwerveDriveOdometry odometry;

  // Limelight
  private boolean targeted;

  // Snap To Angle Algorithm Variables
  private final PIDController snapToAnglePID;
  private double snapToAngleHeading;
  private boolean snapped;

  // Autonomous
  private double teleOpAngleOffset;

  public Drivetrain() {
    // Initialize Swerve Modules
    frontLeftSwerveModule = new MAXSwerveModule(
      RobotMapH.kFrontLeftDrivingCanId,
      RobotMapH.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

    frontRightSwerveModule = new MAXSwerveModule(
      RobotMapH.kFrontRightDrivingCanId,
      RobotMapH.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);
  
    backLeftSwerveModule = new MAXSwerveModule(
      RobotMapH.kRearLeftDrivingCanId,
      RobotMapH.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);
  
    backRightSwerveModule = new MAXSwerveModule(
      RobotMapH.kRearRightDrivingCanId,
      RobotMapH.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

    
    swerveModules = new MAXSwerveModule[] { frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule };
    swerveModulePositions = new SwerveModulePosition[] { frontLeftSwerveModule.getPosition(), frontRightSwerveModule.getPosition(), backLeftSwerveModule.getPosition(),
      backRightSwerveModule.getPosition() };

    // Gyroscope Sensor
    gyro = new ADIS16470_IMU();

    // Teleop Angle offset from Autonomous to Teleop
    teleOpAngleOffset = 0;

    // Snap to Angle Algorithm
    snapToAngleHeading = 0;
    snapped = false;
    snapToAnglePID = new PIDController(DriveConstants.kSnapToAnglePID[0], DriveConstants.kSnapToAnglePID[1], DriveConstants.kSnapToAnglePID[2]);


    // Swerve drive odometry
    odometry = new SwerveDriveOdometry(DriveConstants.kinematics, getRotation2d(), 
    new SwerveModulePosition[] {
      frontLeftSwerveModule.getPosition(),
      frontRightSwerveModule.getPosition(),
      backLeftSwerveModule.getPosition(),
      backRightSwerveModule.getPosition()
    }, new Pose2d(0,0, new Rotation2d()));
      
    // Limelight
    targeted = false;

  }

  @Override
  public void periodic() {
    // Updating the odometry
    for(int i = 0; i < 4; i++){
      swerveModulePositions[i]=swerveModules[i].getPosition();
    }
    updateOdometry();

    for (MAXSwerveModule module : swerveModules) {
      module.putSmartDashboard();
    }
  }

  public static Drivetrain getInstance() {
    if (drivetrain == null) {
      drivetrain = new Drivetrain();
    }
    return drivetrain;
  }

  // Returns the current pose of the robot
  public Pose2d getPose() {
    Pose2d pose = odometry.getPoseMeters();
    return pose;
  }

  // Resets the current pose of the robot
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), swerveModulePositions, pose);
  }

  // Odometry
  public void updateOdometry() {
    odometry.update(getRotation2d(), swerveModulePositions);
  }

  // Drive algorithm
  public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRotation) {
    ChassisSpeeds speeds;

    if (fieldOriented) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, Rotation2d.fromDegrees(teleOpAngleOffset + getHeading()));
    } else {
      speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds, centerOfRotation);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    setSwerveModuleStates(swerveModuleStates);

  }

  public void stopSwerveModules() {
    for(MAXSwerveModule module: swerveModules){
      module.stop();
    }
  }

  public void resetEncoders() {
    frontLeftSwerveModule.resetEncoders();
    backLeftSwerveModule.resetEncoders();
    frontRightSwerveModule.resetEncoders();
    backLeftSwerveModule.resetEncoders();
  }

  public MAXSwerveModule[] getSwerveModules() {
    return swerveModules;
  }

  // Gyroscope
  public double getHeading() {
    heading = gyro.getAngle();
    return Math.IEEEremainder(heading, 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void resetGyro() {
    gyro.reset();
  }
  
  public double getGyroRate(){
    return gyro.getRate();
  }

  public IMUAxis getYaw() {
    return gyro.getYawAxis();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  // Autonomous Transformation
  public void setTeleOpAngleOffset(double target) {
    teleOpAngleOffset = target;
  }

  // Snap to angle algorithm
  private double snapToAngle(){
    // If a button is pressed, use the keepAngle pid contrller to calculate a rotational output so that the robot stays at snapAngle heading
    // If the button is not pressed, use the normal rotation controller
    double output = 0;
    if (Math.abs(getRotation2d().getDegrees() - snapToAngleHeading) > 1) {
      output = snapToAnglePID.calculate(getHeading(), snapToAngleHeading);
    }
    return output;

  }

  public boolean getSnapped(){
    return snapped;
  }

  public void setSnapped(boolean snapped){
    this.snapped = snapped;
  }

  // Limelight
  public boolean isTargeted(){
    return targeted;
  }

  public void setTargeted(boolean lockedOn){
    targeted = lockedOn;
  }

}
