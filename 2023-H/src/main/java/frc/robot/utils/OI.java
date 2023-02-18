package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    private final SlewRateLimiter slewX = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter slewY = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(DriveConstants.kRotationSlewRate);

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

        // Score L1 with any gamepiece
        Trigger xButton = new JoystickButton(driverController, PS4Controller.Button.kCross.value);
        //xButton.onTrue(new SetLevelOnePose());

        // Currently no function.
        Trigger shareButton = new JoystickButton(driverController, PS4Controller.Button.kShare.value);

        // Cube intake / score L1 with any gamepiece
        Trigger optionsButton = new JoystickButton(driverController, PS4Controller.Button.kOptions.value);
        optionsButton.whileTrue(new InstantCommand(() -> setDriveSpeedMode(DriveSpeedMode.SLOW)))
        .onFalse(new InstantCommand(() -> setDriveSpeedMode(DriveSpeedMode.NORMAL)));

        // Level 2 Scoring
        Trigger circleButton = new JoystickButton(driverController, PS4Controller.Button.kCircle.value);
        // circleButton.onTrue(new ConditionalCommand(new SetLevelTwoConePose(), new SetLevelTwoCubePose(), claw::hasCone));

        Trigger squareButton = new JoystickButton(driverController, PS4Controller.Button.kSquare.value);
        // squareButton.onTrue(new SetHumanPlayerConePone());

        Trigger triangleButton = new JoystickButton(driverController, PS4Controller.Button.kTriangle.value);
        // triangleButton.onTrue(new ConditionalCommand(new SetLevelThreeConePose(), new SetLevelThreeCubePose(), claw::hasCone));

        Trigger leftBumperButton = new JoystickButton(driverController, PS4Controller.Button.kL1.value);
        // leftBumperButton.onTrue(new ConditionalCommand(new EjectGamepiece(), new SequentialCommandGroup(new SetFloorConePose(), new IntakeCone()), claw::hasGamepiece));

        Trigger rightBumperButton = new JoystickButton(driverController, PS4Controller.Button.kR1.value);
        // rightBumperButton.onTrue(new ConditionalCommand(new EjectGamepiece(), new SequentialCommandGroup(new SetFloorCubePose(), new IntakeCube()), claw::hasGamepiece));

        Trigger leftStickButton = new JoystickButton(driverController, PS4Controller.Button.kL3.value);
        // leftStickButton.toggleOnTrue(new LockDrivetrain());

        Trigger rightStickButton = new JoystickButton(driverController, PS4Controller.Button.kR3.value);
        // TODO: runs auto-align/driver assist

        // LL seek pose
        Trigger touchpadButton = new JoystickButton(driverController, PS4Controller.Button.kTouchpad.value);
        //touchpadButton.onTrue(new SetLLSeek());

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
        double forwardAxis = getForward();
        double strafeAxis = getStrafe();

        Translation2d next_translation = new Translation2d(slewX.calculate(forwardAxis), slewY.calculate(strafeAxis));

        SmartDashboard.putNumber("raw input forward", forwardAxis);
        SmartDashboard.putNumber("raw input strafe", strafeAxis);
        SmartDashboard.putString("Drive mode", driveSpeedMode.toString());

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

        double combinedRotation = slewRot.calculate((rightRotation-leftRotation)/2.0);
        
        return combinedRotation * getRotationSpeedCoeff() * DriveConstants.kMaxAngularSpeed;
        // return 0;
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