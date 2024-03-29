package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.ADIS16470_IMU;
import frc.robot.utils.RobotMap;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.ADIS16470_IMU.IMUAxis;

public class Drivetrain extends SubsystemBase {

    private static Drivetrain drivetrain;

    private final LimelightFront limelightFront;
    private final LimelightBack limelightBack;

    private BuiltInAccelerometer mRioAccel;

    // Swerve Modules
    private final MAXSwerveModule[] swerveModules;
    private final MAXSwerveModule frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule,
            backRightSwerveModule;

    // Gyroscope Sensor
    private final ADIS16470_IMU gyro;
    private double heading;
    private boolean isFlipped;

    // Swerve Drive
    private SwerveModuleState[] swerveModuleStates;
    private SwerveModulePosition[] swerveModulePositions;

    private double latestChassisSpeed;

    private final SwerveDrivePoseEstimator odometry;

    // Snap To Angle Algorithm Variables
    private final PIDController snapToAnglePID;
    private double snapToAngleHeading;
    private boolean snapped;

    // Correct angle algorithm variables
    private boolean useHeadingCorrection;
    private Rotation2d correctHeadingTargetHeading;
    private Timer correctHeadingTimer;
    private double correctHeadingPreviousTime;
    private double correctHeadingOffTime;

    // Global drivetrain teleop toggle
    private boolean allowDriving;

    // Gyro Tilt Rolling Average
    private RollingAverage gyroTiltAverage;

    public Drivetrain() {
        limelightFront = LimelightFront.getInstance();
        limelightBack = LimelightBack.getInstance();


        // Initialize Swerve Modules
        frontLeftSwerveModule = new MAXSwerveModule(
                RobotMap.kFrontLeftDrivingCanId,
                RobotMap.kFrontLeftTurningCanId,
                DriveConstants.kFrontLeftChassisAngularOffset);

        frontRightSwerveModule = new MAXSwerveModule(
                RobotMap.kFrontRightDrivingCanId,
                RobotMap.kFrontRightTurningCanId,
                DriveConstants.kFrontRightChassisAngularOffset);

        backLeftSwerveModule = new MAXSwerveModule(
                RobotMap.kRearLeftDrivingCanId,
                RobotMap.kRearLeftTurningCanId,
                DriveConstants.kBackLeftChassisAngularOffset);

        backRightSwerveModule = new MAXSwerveModule(
                RobotMap.kRearRightDrivingCanId,
                RobotMap.kRearRightTurningCanId,
                DriveConstants.kBackRightChassisAngularOffset);

        swerveModules = new MAXSwerveModule[] { frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule,
                backRightSwerveModule };
        swerveModulePositions = new SwerveModulePosition[] { frontLeftSwerveModule.getPosition(),
                frontRightSwerveModule.getPosition(), backLeftSwerveModule.getPosition(),
                backRightSwerveModule.getPosition() };

        // Gyroscope Sensor
        gyro = new ADIS16470_IMU();
        isFlipped = false;

        // Snap to Angle Algorithm
        snapToAngleHeading = 0;
        snapped = false;
        snapToAnglePID = new PIDController(DriveConstants.kSnapToAnglePID[0], DriveConstants.kSnapToAnglePID[1],
                DriveConstants.kSnapToAnglePID[2]);

        // Correct heading algorithm
        useHeadingCorrection = true;
        correctHeadingTimer = new Timer();
        correctHeadingTimer.start();
        correctHeadingPreviousTime = 0.0;
        correctHeadingOffTime = 0.0;

        // Swerve drive odometry
        odometry = new SwerveDrivePoseEstimator(DriveConstants.kinematics, getHeadingAsRotation2d(),
                new SwerveModulePosition[] {
                        frontLeftSwerveModule.getPosition(),
                        frontRightSwerveModule.getPosition(),
                        backLeftSwerveModule.getPosition(),
                        backRightSwerveModule.getPosition() },
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, 0.9));

        allowDriving = true;

        latestChassisSpeed = 0.0;
        correctHeadingTargetHeading = getHeadingAsRotation2d();

        gyroTiltAverage = new RollingAverage();

        SmartDashboard.putBoolean("isFlipped", isFlipped);

        SmartDashboard.putNumber("drive p", ModuleConstants.kDrivingP);
        SmartDashboard.putNumber("drive i", ModuleConstants.kDrivingI);
        SmartDashboard.putNumber("drive d", ModuleConstants.kDrivingD);
        SmartDashboard.putNumber("drive ff", ModuleConstants.kDrivingFF);

        SmartDashboard.putNumber("turn p", ModuleConstants.kTurningP);
        SmartDashboard.putNumber("turn i", ModuleConstants.kTurningI);
        SmartDashboard.putNumber("turn d", ModuleConstants.kTurningD);
        SmartDashboard.putNumber("turn ff", ModuleConstants.kTurningFF);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IMU Pitch", getPitch());
        SmartDashboard.putNumber("IMU Pitch Average", gyroTiltAverage.getAverage());
        SmartDashboard.putBoolean("isFlipped", isFlipped);

        // Updating the odometry
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = swerveModules[i].getPosition();
            swerveModules[i].putSmartDashboard();
            // swerveModules[i].setDrivePIDF(SmartDashboard.getNumber("drive p", 0.0),
            //     SmartDashboard.getNumber("drive i", 0.0),
            //     SmartDashboard.getNumber("drive d", 0.0),
            //     SmartDashboard.getNumber("drive ff", 0.0));
            // swerveModules[i].setTurnPIDF(SmartDashboard.getNumber("turn p", 0.0),
            // SmartDashboard.getNumber("turn i", 0.0),
            // SmartDashboard.getNumber("turn d", 0.0),
            // SmartDashboard.getNumber("turn ff", 0.0));
        }

        updateOdometry();

        gyroTiltAverage.add(getPitch());

    }

    public static Drivetrain getInstance() {
        if (drivetrain == null) {
            drivetrain = new Drivetrain();
        }
        return drivetrain;
    }

    public double getSpeed(){
        return latestChassisSpeed;
    }

    public void setAllowDriving(boolean allow){
        allowDriving = allow;
    }

    public void setUseHeadingCorrection(boolean enable){
      useHeadingCorrection = enable;
    }

    public SwerveModuleState[] getSwerveModuleStates(){
      return swerveModuleStates;
    }

    // Returns the current pose of the robot
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public Translation2d getPoseAsTranslation2d() {
        return odometry.getEstimatedPosition().getTranslation();
    }

    // Resets the current pose of the robot
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getHeadingAsRotation2d(), swerveModulePositions, pose);
    }

    // Resets the current pose and heading of the robot
    public void resetRobotPoseAndGyro(Pose2d pose) {
        resetGyro();
        odometry.resetPosition(getHeadingAsRotation2d(), swerveModulePositions, pose);
    }

    // Odometry
    public void updateOdometry() {
        odometry.updateWithTime(Timer.getFPGATimestamp(), getHeadingAsRotation2d(), swerveModulePositions);

        // limelightFront.checkForAprilTagUpdates(odometry);
        // limelightBack.checkForAprilTagUpdates(odometry);
    }

    public void setFlipped(){
        isFlipped = Math.abs(getPose().getRotation().getDegrees()) < 90;
    }

    public void setFlipped(boolean bool){
        isFlipped = bool;
    }
    
    public boolean getFlipped(){
        return isFlipped;
    }

    // Drive algorithm
    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kRealMaxSpeedMetersPerSecond);

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(
            frontLeftSwerveModule.getState(), frontRightSwerveModule.getState(),
            backLeftSwerveModule.getState(), backRightSwerveModule.getState()
        );
    }

    public void driveAuton(ChassisSpeeds speeds) {
        swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        setSwerveModuleStates(swerveModuleStates);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented,
            Translation2d centerOfRotation) {
        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        ChassisSpeeds robotRelativeSpeeds;

        if (useHeadingCorrection) {
            fieldRelativeSpeeds = correctHeading(fieldRelativeSpeeds);
        }

        if (fieldOriented) {
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds,
                    Rotation2d.fromDegrees((isFlipped ? 180 : 0) + getHeading()));
        } else {
            robotRelativeSpeeds = fieldRelativeSpeeds;
        }

        latestChassisSpeed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2) + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));

        swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds, centerOfRotation);

        if(allowDriving){
            setSwerveModuleStates(swerveModuleStates);
        }
    }

    public void stopSwerveModules() {
        for (MAXSwerveModule module : swerveModules) {
            module.stop();
        }
    }

    /**
     * This function optimizes the chassis speed that is put into the kinematics
     * object to allow the robot to hold its heading
     * when no angular velocity is input. The robot will therefore correct itself
     * when it turns without us telling it to do so.
     *
     * @param desiredSpeed Desired chassis speed that is input by the controller
     * @return {@code correctedChassisSpeed} which takes into account that the robot
     *         needs to have the same heading when no rotational speed is input
     */
    private ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed) {

        // Determine time interval
        double correctHeadingCurrentTime = correctHeadingTimer.get();
        double dt = correctHeadingCurrentTime - correctHeadingPreviousTime;

        // Get desired rotational speed in radians per second and absolute translational
        // speed in m/s
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.sqrt(Math.pow(desiredSpeed.vxMetersPerSecond, 2) + Math.pow(desiredSpeed.vxMetersPerSecond, 2));

        if (vr > 0.01 || vr < -0.01) {
            correctHeadingOffTime = correctHeadingCurrentTime;
            correctHeadingTargetHeading = getHeadingAsRotation2d();
            return desiredSpeed;
        }
        if (correctHeadingCurrentTime - correctHeadingOffTime < 0.5) {
            correctHeadingTargetHeading = getHeadingAsRotation2d();
            return desiredSpeed;
        }
        if (v < 0.05) {
            correctHeadingTargetHeading = getHeadingAsRotation2d();
            return desiredSpeed;
        }

        // Determine target and current heading
        correctHeadingTargetHeading = correctHeadingTargetHeading.plus(new Rotation2d(vr * dt));
        Rotation2d currentHeading = getHeadingAsRotation2d();

        // Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = correctHeadingTargetHeading.minus(currentHeading);

        if (Math.abs(deltaHeading.getDegrees()) < DriveConstants.kHeadingCorrectionTolerance) {
            return desiredSpeed;
        }
        double correctedVr = deltaHeading.getRadians() / dt * DriveConstants.kHeadingCorrectionP;

        correctHeadingPreviousTime = correctHeadingCurrentTime;

        return new ChassisSpeeds(
                desiredSpeed.vxMetersPerSecond,
                desiredSpeed.vyMetersPerSecond,
                correctedVr);
    }

    // Return a vector representing the displacement between the current (x,y) pose and some other point.
    public Translation2d getNormalizedTranslationToPoint(Translation2d otherCoord) {
        Translation2d currentXY = getPoseAsTranslation2d();
        Translation2d moveXY = new Translation2d(-(otherCoord.getX() - currentXY.getX()),
            -(otherCoord.getY() - currentXY.getY()));
        // finds the distance between the current odometry thing and the given coord
        Translation2d XY = moveXY.div(moveXY.getNorm());
        return XY;
    }

    public Translation2d getTranslationToPoint(Translation2d otherCoord, double scalar) {
        Translation2d currentXY = getPoseAsTranslation2d();
        Translation2d moveXY = new Translation2d(-(otherCoord.getX() - currentXY.getX()),
            -(otherCoord.getY() - currentXY.getY()));
        // finds the distance between the current odometry thing and the given coord
        Translation2d XY = moveXY.times(scalar);
        return XY;
    }

    public Translation2d getTranslationToPoint(Pose2d otherCoord, double scalar) {
        Translation2d currentXY = getPoseAsTranslation2d();
        Translation2d moveXY = new Translation2d(-(otherCoord.getX() - currentXY.getX()),
            -(otherCoord.getY() - currentXY.getY()));
        // finds the distance between the current odometry thing and the given coord
        Translation2d XY = moveXY.times(scalar);
        return XY;
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
        heading = gyro.getAngle(gyro.getYawAxis());
        return Math.IEEEremainder(heading, 360);
    }

    public Rotation2d getHeadingAsRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetGyro() {
        gyro.resetAllAngles();
        isFlipped = true;
        correctHeadingTimer.reset();
        correctHeadingTimer.start();
    }

    public double getPitch() {
        return gyro.getAngle(IMUAxis.kRoll);
    }

    public double getPitchRate(){
        return gyro.getRate(IMUAxis.kRoll);
    }

    public double getPitchAverage(){
        return gyroTiltAverage.getAverage();
    }

    public boolean getSnapped() {
        return snapped;
    }

    public void setSnapped(boolean snapped) {
        this.snapped = snapped;
    }

    public void lock() {
        frontLeftSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)));
        backLeftSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)));
        frontRightSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)));
        backRightSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(-3 * Math.PI / 4)));
    }

    public void straighten() {
        frontLeftSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        backLeftSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        frontRightSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        backRightSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
    }

    public double[] getModuleRotations(){
        double[] positions = {frontLeftSwerveModule.getRotations(),
            backLeftSwerveModule.getRotations(),
            frontRightSwerveModule.getRotations(),
            backRightSwerveModule.getRotations()};
        return (positions);
    }

    public SwerveDrivePoseEstimator getOdometry() {
        return odometry;
    }


}
