package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shuffleboard;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.commands.ArmCommands.SetFloorConePose;
import frc.robot.commands.ArmCommands.SetFloorCubePose;
import frc.robot.commands.ArmCommands.SetHumanPlayerConePose;
import frc.robot.commands.ArmCommands.SetLevelOnePose;
import frc.robot.commands.ArmCommands.SetLevelThreeConePose;
import frc.robot.commands.ArmCommands.SetLevelThreeCubePose;
import frc.robot.commands.ArmCommands.SetLevelTwoConePose;
import frc.robot.commands.ArmCommands.SetLevelTwoCubePose;
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.commands.ClawCommands.EjectGamepiece;
import frc.robot.commands.ClawCommands.IntakeCone;
import frc.robot.commands.ClawCommands.IntakeCube;
import frc.robot.commands.DriveCommands.LockDrivetrain;

public class OI {
    public static OI instance;

    private Drivetrain drivetrain;
    private Claw claw;
    private Shuffleboard shuffleboard;

    private PS4Controller driverController = new PS4Controller(0);

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    public enum DPadDirection {NONE, FORWARDS, LEFT, RIGHT, BACKWARDS};
    public enum DriveSpeedMode{NORMAL, SLOW};

    private DriveSpeedMode driveSpeedMode;

    public OI() {
        drivetrain = Drivetrain.getInstance();
        claw = Claw.getInstance();
        shuffleboard = Shuffleboard.getInstance();

        driveSpeedMode = DriveSpeedMode.NORMAL;

        setupControls();
        
    }

    public void setupControls(){
        // Drive & Scoring Controls

        // Score L1 with any gamepiece
        Trigger xButton = new JoystickButton(driverController, PS4Controller.Button.kCross.value);
        xButton.onTrue(new SetLevelOnePose());

        // Cube intake / score L1 with any gamepiece
        Trigger leftBumperButton = new JoystickButton(driverController, PS4Controller.Button.kL1.value);
        leftBumperButton.onTrue(new ConditionalCommand(new EjectGamepiece(), new SequentialCommandGroup(new SetFloorConePose(), new IntakeCone()), claw::hasGamepiece));

        Trigger rightBumperButton = new JoystickButton(driverController, PS4Controller.Button.kR1.value);
        rightBumperButton.onTrue(new ConditionalCommand(new EjectGamepiece(), new SequentialCommandGroup(new SetFloorCubePose(), new IntakeCube()), claw::hasGamepiece));

        // Level 2 Scoring
        Trigger circleButton = new JoystickButton(driverController, PS4Controller.Button.kCircle.value);
        circleButton.onTrue(new ConditionalCommand(new SetLevelTwoConePose(), new SetLevelTwoCubePose(), claw::hasCone));

        // Level 3 Scoring
        Trigger triangleButton = new JoystickButton(driverController, PS4Controller.Button.kTriangle.value);
        // triangleButton.onTrue(new ConditionalCommand(new SetLevelThreeConePose(), new SetLevelThreeCubePose(), claw::hasCone));

        // Currently no function.
        Trigger shareButton = new JoystickButton(driverController, PS4Controller.Button.kShare.value);

       // Align to goal
        Trigger rightStickButton = new JoystickButton(driverController, PS4Controller.Button.kR3.value);
        // TODO: runs auto-align/driver assist

        // MISC CONTROLS

        // Lock Drivetrain
        Trigger leftStickButton = new JoystickButton(driverController, PS4Controller.Button.kL3.value);
        // leftStickButton.toggleOnTrue(new LockDrivetrain());

        // LL seek pose
        Trigger touchpadButton = new JoystickButton(driverController, PS4Controller.Button.kTouchpad.value);
        //touchpadButton.onTrue(new SetLLSeek());

        // Human Player Station Loading
        Trigger squareButton = new JoystickButton(driverController, PS4Controller.Button.kSquare.value);
        // squareButton.onTrue(new SetHumanPlayerConePone());

        // Slow Mode
        Trigger optionsButton = new JoystickButton(driverController, PS4Controller.Button.kOptions.value);
        optionsButton.whileTrue(new InstantCommand(() -> setDriveSpeedMode(DriveSpeedMode.SLOW))).onFalse(new InstantCommand(() -> setDriveSpeedMode(DriveSpeedMode.NORMAL)));

        // Reset gyro (resets field oriented drive)
        Trigger ps4Button = new JoystickButton(driverController, PS4Controller.Button.kPS.value);
        ps4Button.onTrue(new InstantCommand(() -> drivetrain.resetGyro()));
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public void setDriveSpeedMode(DriveSpeedMode mode){
        driveSpeedMode = mode;
    }

    public double getForward() {
        return driverController.getRawAxis(PS4Controller.Axis.kLeftY.value);
        // return 0;
    }

    public double getStrafe() {
        return driverController.getRawAxis(PS4Controller.Axis.kLeftX.value);
        // return 0;
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double xSpeed = getForward();
        double ySpeed = getStrafe();

        SmartDashboard.putNumber("raw input forward", xSpeed);
        SmartDashboard.putNumber("raw input strafe", ySpeed);
        SmartDashboard.putString("Drive mode", driveSpeedMode.toString());

        double xSpeedCommanded;
        double ySpeedCommanded;
    
        if (DriveConstants.kUseRateLimit) {
          // Convert XY to polar for rate limiting
          double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
          double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
    
          // Calculate the direction slew rate based on an estimate of the lateral acceleration
          double directionSlewRate;
          if (m_currentTranslationMag != 0.0) {
            directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
          } else {
            directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
          }
          
          double currentTime = WPIUtilJNI.now() * 1e-6;
          double elapsedTime = currentTime - m_prevTime;
          double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
          if (angleDif < 0.45*Math.PI) {
            m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
          }
          else if (angleDif > 0.85*Math.PI) {
            if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
              // keep currentTranslationDir unchanged
              m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            else {
              m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
              m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            }
          }
          else {
            m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_magLimiter.calculate(0.0);
          }
          m_prevTime = currentTime;
          
          xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
          ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);    
    
        } else {
          xSpeedCommanded = xSpeed;
          ySpeedCommanded = ySpeed;
        }

        Translation2d next_translation = new Translation2d(xSpeedCommanded, ySpeedCommanded);

        double norm = next_translation.getNorm();
        if (norm < OIConstants.kDrivingDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(next_translation.getX(), next_translation.getY());     
            Translation2d deadband_vector = fromPolar(deadband_direction, OIConstants.kDrivingDeadband);

            double new_translation_x = next_translation.getX() - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double new_translation_y = next_translation.getY() - (deadband_vector.getY()) / (1 - deadband_vector.getY());

            next_translation = new Translation2d(new_translation_x * getTranslationSpeedCoeff() * DriveConstants.kMaxFloorSpeed,
            new_translation_y * getTranslationSpeedCoeff()  * DriveConstants.kMaxFloorSpeed);
            
            return next_translation;
        }
    }

    public double getTranslationSpeedCoeff(){
        if(driveSpeedMode == DriveSpeedMode.SLOW){
            return DriveConstants.kSlowModeTranslationSpeedScale;
        }
        else{
            return DriveConstants.kNormalModeTranslationSpeedScale;
        }
    }

    public double getRotationSpeedCoeff(){
        if(driveSpeedMode == DriveSpeedMode.SLOW){
            return DriveConstants.kSlowModeRotationSpeedScale;
        }
        else{
            return DriveConstants.kNormalModeRotationSpeedScale;
        }
    }

    public Translation2d fromPolar(Rotation2d direction, double magnitude){
    	return new Translation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

    public double getRotation() {
        double leftRotation = driverController.getRawAxis(PS4Controller.Axis.kL2.value);
        double rightRotation = driverController.getRawAxis(PS4Controller.Axis.kR2.value);

        double combinedRotation;
        if(DriveConstants.kUseRateLimit){
            combinedRotation = m_rotLimiter.calculate((rightRotation-leftRotation)/2.0);
        }
        else{
            combinedRotation = (rightRotation-leftRotation)/2.0;
        }

        return combinedRotation * getRotationSpeedCoeff() * DriveConstants.kMaxAngularSpeed;
    }

    public Translation2d getCenterOfRotation() {
        double rotX = driverController.getRawAxis(2) * DriveConstants.kWheelbase;
        double rotY = driverController.getRawAxis(5) * DriveConstants.kTrackwidth;

        if (rotX * rotY > 0) {
            rotX = -rotX;
            rotY = -rotY;
        }
        rotX *= 0.75;
        rotY *= 0.75;
        Translation2d output = new Translation2d(rotX, rotY);
        return output;
    }

    public DPadDirection getDriverDPadInput(){
        switch (driverController.getPOV()) {
            case 0:
                return DPadDirection.FORWARDS;
            case 90:
                return DPadDirection.RIGHT;
            case 270:
                return DPadDirection.LEFT;
            case 180:
                return DPadDirection.BACKWARDS;
            default:
                return DPadDirection.NONE;
        }
    }

    public Translation2d getCardinalDirection(){
        switch (getDriverDPadInput()) {
            case FORWARDS:
                return new Translation2d(DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed, 0.0);
            case RIGHT:
                return new Translation2d(0.0, -DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed);
            case LEFT:
                return new Translation2d(0.0, DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed);
            case BACKWARDS:
                return new Translation2d(-DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed, 0.0);
            default:
                return new Translation2d(0.0, 0.0);
        }

    }

    // For testing purposes/ open loop mode.
    public double getArmSpeed(){
        if(Math.abs(driverController.getRawAxis(PS4Controller.Axis.kRightY.value)) > 0.01){
            return -driverController.getRawAxis(PS4Controller.Axis.kRightY.value)*0.6;
        }
        return 0;
    }

    public double signedSquared(double input) {
        return Math.signum(input) * Math.pow(input, 2);
    }

    public double applyDeadband(double input) {
        if (Math.abs(input) < OIConstants.kDrivingDeadband) {
            return 0.0;
        }
        return input;
    }

    public double inputTransform(double input) {
        return signedSquared(applyDeadband(input));
    }
}