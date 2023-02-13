package frc.robot.utils;

import javax.sql.rowset.spi.TransactionalWriter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.commands.ClawCommands.ConeIntake;
import frc.robot.commands.ClawCommands.CubeIntake;
import frc.robot.commands.ClawCommands.RunClawSmartDashboard;
import frc.robot.commands.DriveCommands.LockDrivetrain;
import frc.robot.subsystems.Claw;

public class OI {
    public static OI instance;
    private Drivetrain drivetrain;

    private PS4Controller driverController = new PS4Controller(0);

    private final SlewRateLimiter slewX = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter slewY = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(DriveConstants.kRotationSlewRate);

    public enum DPadDirection {NONE, FORWARDS, LEFT, RIGHT, BACKWARDS};
    public enum DriveSpeedMode{NORMAL, SLOW};

    private DriveSpeedMode driveSpeedMode;

    public OI() {
        drivetrain = Drivetrain.getInstance();

        driveSpeedMode = DriveSpeedMode.NORMAL;

        setupControls();
        
    }

    public void setupControls(){
        Trigger resetGyroButton = new JoystickButton(driverController, 1);
        resetGyroButton.whileTrue(new InstantCommand(() -> drivetrain.resetGyro()));

        Trigger lockWheels = new JoystickButton(driverController, 2);
        lockWheels.whileTrue(new LockDrivetrain());

        Trigger coneIntake = new JoystickButton(driverController, 3);
        coneIntake.whileTrue(new ConeIntake());

        Trigger cubeIntake = new JoystickButton(driverController, 4);
        cubeIntake.whileTrue(new CubeIntake());

        Trigger slowMode = new JoystickButton(driverController, 5);
        slowMode.whileTrue(new InstantCommand(() -> setDriveSpeedMode(DriveSpeedMode.SLOW)));

        // Trigger snapToAngle = new JoystickButton(driverPs4Controller, 5);
        // snapToAngle.whileTrue(new SnapToAngle());

        Trigger xButton = new JoystickButton(driverController, PS4Controller.Button.kCross.value);
        // TODO: ejects game piece
        xButton.onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("Circle Pressed", true)));
        xButton.onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Circle Pressed", false)));

        Trigger circleButton = new JoystickButton(driverController, PS4Controller.Button.kCircle.value);
        // TODO: runs level 2 scoring pose function

        Trigger squareButton = new JoystickButton(driverController, PS4Controller.Button.kSquare.value);
        // TODO: runs human station intake pose function

        Trigger triangleButton = new JoystickButton(driverController, PS4Controller.Button.kTriangle.value);
        // TODO: runs level 3 scoring pose function

        Trigger leftBumperButton = new JoystickButton(driverController, PS4Controller.Button.kL1.value);
        // TODO: changes to slow mode

        Trigger rightBumperButton = new JoystickButton(driverController, PS4Controller.Button.kR1.value);
        // TODO: runs auto-aligner/driver assist

        Trigger leftStickButton = new JoystickButton(driverController, PS4Controller.Button.kL3.value);
        // TODO: runs X lock (safe pose)

        Trigger rightStickButton = new JoystickButton(driverController, PS4Controller.Button.kR3.value);
        // TODO: runs rotations lock

        Trigger backButton = new JoystickButton(driverController, PS4Controller.Button.kShare.value);
        // TODO: runs cone floor intake/level 1 scoring pose

        Trigger startButton = new JoystickButton(driverController, PS4Controller.Button.kOptions.value);
        // TODO: runs cube floor intake/level 1 scoring pose

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
        // return driverController.getRawAxis(PS4Controller.Axis.kLeftY.value);
        return 0;
    }

    public double getStrafe() {
        // return driverController.getRawAxis(PS4Controller.Axis.kLeftX.value);
        return 0;
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = getForward();
        double strafeAxis = getStrafe();

        Translation2d next_translation = new Translation2d(slewX.calculate(forwardAxis), slewY.calculate(strafeAxis));

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
            
            SmartDashboard.putNumber("field relative input forward axis", next_translation.getX());
            SmartDashboard.putNumber("field relative input strafe axis", next_translation.getY());
    
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
        // double leftRotation = driverController.getRawAxis(PS4Controller.Axis.kL2.value);
        // double rightRotation = driverController.getRawAxis(PS4Controller.Axis.kR2.value);
        // double combinedRotation = slewRot.calculate((rightRotation-leftRotation)/2.0);
        // return combinedRotation * getRotationSpeedCoeff() * DriveConstants.kMaxAngularSpeed;
        return 0;
    }

    public Translation2d getCenterOfRotation() {
        // double rotX = driverController.getRawAxis(2) * DriveConstants.kWheelbase;
        // double rotY = driverController.getRawAxis(5) * DriveConstants.kTrackwidth;

        // if (rotX * rotY > 0) {
        //     rotX = -rotX;
        //     rotY = -rotY;
        // }
        // rotX *= 0.75;
        // rotY *= 0.75;
        // Translation2d output = new Translation2d(rotX, rotY);
        // return output;
        return new Translation2d(0, 0);
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
        // Need to switch out "kMaxSpeedMetersPerSecond" for max real floor speed when merged
        switch (getDriverDPadInput()) {
            case FORWARDS:
                return new Translation2d(DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed, 0.0);
            case RIGHT:
                return new Translation2d(0.0, -0.3 * DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed);
            case LEFT:
                return new Translation2d(0.0, 0.3 * DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed);
            case BACKWARDS:
                return new Translation2d(-0.3 * DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed, 0.0);
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