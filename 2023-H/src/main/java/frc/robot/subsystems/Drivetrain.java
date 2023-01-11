package frc.robot.subsystems;

import javax.sql.rowset.spi.TransactionalWriter;

// import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.RobotMapH;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;

  private final PIDController keepAnglePID = new PIDController(DriveConstants.kKeepAnglePID[0],
      DriveConstants.kKeepAnglePID[1], DriveConstants.kKeepAnglePID[2]);

  private double keepAngle = 0.0;
  private double timeSinceRot = 0.0;
  private double lastRotTime = 0.0;
  private double timeSinceDrive = 0.0;
  private double lastDriveTime = 0.0;

  private final Timer keepAngleTimer = new Timer();

  /** Angle Offsets */
  private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0); // external magnet encoder - internal
                                                                              // motor encoder
  private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0); // external magnet encoder - internal
                                                                               // motor encoder
  private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0); // external magnet encoder - internal motor
                                                                             // encoder
  private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0); // external magnet encoder - internal
                                                                              // motor encoder

  private final SwerveModule[] swerveModules;
  private final SwerveModule frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule;

  private final ADIS16470_IMU gyro;
  private double headingValue;

  private boolean brakeMode;

  private SwerveModuleState[] swerveModuleStates;
  private SwerveModulePosition[] swerveModulePositions;

  private double teleOpAngleOffset;

  private final SwerveDriveOdometry odometry;

  private boolean targeted;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    frontLeftSwerveModule = new SwerveModule(RobotMapH.frontLeftSwerveModuleCID, RobotMapH.frontLeftSwerveModuleAID, DriveConstants.kFrontLeftDriveP, DriveConstants.kFrontLeftAngleP,
        DriveConstants.kFrontLeftTurningEncoderChannel, DriveConstants.kFrontLeftAngularOffset);// last 2 are random
                                                                                                // vals. They are
                                                                                                // canCoder ID and
                                                                                                // offset angle in
                                                                                                // radians
    frontRightSwerveModule = new SwerveModule(RobotMapH.frontRightSwerveModuleCID, RobotMapH.frontRightSwerveModuleAID, DriveConstants.kFrontRightDriveP,
        DriveConstants.kFrontRightAngleP, DriveConstants.kFrontRightTurningEncoderChannel, DriveConstants.kFrontRightAngularOffset);
                                                                                                // last 2 are random vals. 
                                                                                                //They are canCoder ID 
                                                                                                //and offset angle in
                                                                                                // radians
    backLeftSwerveModule = new SwerveModule(RobotMapH.backLeftSwerveModuleCID, RobotMapH.backLeftSwerveModuleAID, DriveConstants.kBackLeftDriveP, DriveConstants.kBackLeftAngleP,
        DriveConstants.kBackLeftTurningEncoderChannel, DriveConstants.kBackLeftAngularOffset);// last 2 are random vals.
                                                                                              // They are canCoder ID
                                                                                              // and offset angle in
                                                                                              // radians
    backRightSwerveModule = new SwerveModule(RobotMapH.backRightSwerveModuleCID, RobotMapH.backRightSwerveModuleAID, DriveConstants.kBackRightDriveP, DriveConstants.kBackRightAngleP,
        DriveConstants.kBackRightTurningEncoderChannel, DriveConstants.kBackRightAngularOffset);// last 2 are random
                                                                                                // vals. They are
                                                                                                // canCoder ID and
                                                                                                // offset angle in
                                                                                                // radianss
    swerveModules = new SwerveModule[] { frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule,
        backRightSwerveModule };

    gyro = new ADIS16470_IMU();

    teleOpAngleOffset = 0;

    keepAngleTimer.reset();
    keepAngleTimer.start();

    odometry = new SwerveDriveOdometry(DriveConstants.kinematics, getRotation2d(), 
    new SwerveModulePosition[] {
      frontLeftSwerveModule.getCurrentPosition(),
      frontRightSwerveModule.getCurrentPosition(),
      backLeftSwerveModule.getCurrentPosition(),
      backRightSwerveModule.getCurrentPosition()
    }, new Pose2d(0,0, new Rotation2d()));; //constructor is SwerveDriveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPose)
    //odometry.resetPosition(getRotation2d(),swerveModuleStates,new Pose2d());//ODOMETRY
      
    targeted = false;

  }

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRotation) {

    rotation = performKeepAngle(translation.getX(), translation.getY(), rotation);

    ChassisSpeeds speeds;
    // Convert the inputs from the controller into a vector of desired
    // translational/rotational speeds for the robot
    if (fieldOriented) {
      // speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(),
      // translation.getY(), rotation, Rotation2d.fromDegrees(teleOpAngleOffset +
      // getHeading()));
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
          Rotation2d.fromDegrees(teleOpAngleOffset + getHeading()));
    } else {
      speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    SmartDashboard.putNumber("ChassisSpeed x (m/s)", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed y (m/s)", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed rot (r/s)", speeds.omegaRadiansPerSecond);

    swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds, centerOfRotation); // calculate the
                                                                                                   // states of the
                                                                                                   // individual swerve
                                                                                                   // modules from the
                                                                                                   // global vector

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Update each swerve module according to the SwerveModuleStates computed by
    // WPILib
    // for (int i = 0; i < swerveModules.length; i++) {
    // swerveModules[i].setDesiredState(swerveModuleStates[i]);
    // }
    setSwerveModuleStates(swerveModuleStates);

  }

  public void stopSwerveModules() {
    for(SwerveModule swervemodule: swerveModules){
      swervemodule.stop();
    }
  }

  public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
    // Update each swerve module according to the SwerveModuleStates computed by
    // WPILib
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public SwerveModule getFrontLeftSwerveModule() {
    return frontLeftSwerveModule;
  }

  public SwerveModule getFrontRightSwerveModule() {
    return frontRightSwerveModule;
  }

  public SwerveModule getBackLeftSwerveModule() {
    return backLeftSwerveModule;
  }

  public SwerveModule getBackRightSwerveModule() {
    return backRightSwerveModule;
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModules;
  }

  public double getHeading() {
    headingValue = gyro.getAngle();
    return Math.IEEEremainder(headingValue, 360);
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

  public void stop() {
    for (SwerveModule mod : swerveModules) {
      mod.stop();
    }
  }

  public void setTeleOpAngleOffset(double target) {
    teleOpAngleOffset = target;
  }

  public void resetSwerveModules(){
    for(SwerveModule swervemodule: swerveModules){
      swervemodule.resetSwerveModule();
    }
  }

  private double performKeepAngle(double xSpeed, double ySpeed, double rot) {
    double output = rot;
    if (Math.abs(rot) >= DriveConstants.kMinTranslationCommand) {
      lastRotTime = keepAngleTimer.get();
    }
    if (Math.abs(xSpeed) >= DriveConstants.kMinTranslationCommand
        || Math.abs(ySpeed) >= DriveConstants.kMinTranslationCommand) {
      lastDriveTime = keepAngleTimer.get();
    }
    timeSinceRot = keepAngleTimer.get() - lastRotTime;
    timeSinceDrive = keepAngleTimer.get() - lastDriveTime;

    if (timeSinceRot < 0.5) {
      keepAngle = getRotation2d().getRadians();
    } else if (Math.abs(rot) < DriveConstants.kMinRotationCommand && timeSinceDrive < 0.25) {
      output = keepAnglePID.calculate(getRotation2d().getRadians(), keepAngle);
    }

    return output;
  }

  public void updateKeepAngle() {
    keepAngle = getRotation2d().getRadians();
  }

  public void updateOdometry() {
    odometry.update(getRotation2d(), swerveModulePositions);//ODOMETRY
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), swerveModulePositions, pose);//ODOMETRY
  }

  public void resetRobotPose(Pose2d pose) {
    resetGyro();
    odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), swerveModulePositions, pose);//ODOMETRY
  }

  public Pose2d getPose() {
    Pose2d pose = odometry.getPoseMeters();
    return pose;
  }

  public boolean isTargeted(){
    return targeted;
  }

  public void setTargeted(boolean lockedOn){
    targeted = lockedOn;
  }

  @Override
  public void periodic() {
    // Updating the odometry
    for(int i=0;i<4;i++){
      swerveModulePositions[i]=swerveModules[i].getCurrentPosition();
    }
    updateOdometry();

    for (SwerveModule module : swerveModules) {
      module.putShuffleboard();
      module.getFromShuffleboard();
    }
    // SmartDashboard.putNumber("lastRotTime", lastRotTime);
    // SmartDashboard.putNumber("lastDriveTime", lastDriveTime);
    // SmartDashboard.putNumber("timeSinceRot", timeSinceRot);
    // SmartDashboard.putNumber("timeSinceDrive", timeSinceDrive);
    // SmartDashboard.putNumber("keep angle", keepAngle);
    SmartDashboard.putNumber("gyro angle", gyro.getAngle());
    SmartDashboard.putNumber("gyro heading", getHeading());
    SmartDashboard.putNumber("x", getPose().getTranslation().getX());
    SmartDashboard.putNumber("y", getPose().getTranslation().getY());

    

  }

}
