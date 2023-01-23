package frc.robot;

import javax.sql.rowset.spi.TransactionalWriter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MeasurementConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class OI {
    public static OI instance;
    private Drivetrain drivetrain;
    private Intake intake;
    // private Joystick leftJoystick = new Joystick(0);
    // private Joystick rightJoystick = new Joystick(1);

    private Joystick driverXboxController = new Joystick(0); // this is no longer just a random number

    
    private final SlewRateLimiter slewX = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter slewY = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(DriveConstants.kRotationSlewRate);
    private double rotation;

    public OI() {
        drivetrain = Drivetrain.getInstance();
        //Resets the gyro
        Trigger resetGyroButton = new JoystickButton(driverXboxController, 1);
        resetGyroButton.whileTrue(new InstantCommand(() -> drivetrain.resetGyro()));
        // Snap to angle
        Trigger snapToAngle = new JoystickButton(driverXboxController, 5);
        // snapToAngle.whileTrue(new SnapToAngle());
        
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public double getForward() {
        return driverXboxController.getRawAxis(1);
    }

    public double getStrafe() {
        return driverXboxController.getRawAxis(0);
    }

    public Translation2d getTranslation2d(){
        Translation2d position = new Translation2d(
        slewX.calculate(-inputTransform(getForward())) * DriveConstants.kMaxSpeedMetersPerSecond,
        slewY.calculate(-inputTransform(getStrafe())) * DriveConstants.kMaxSpeedMetersPerSecond);

        return position;
    }

    public double getRotation() {
        double leftRotation = driverXboxController.getRawAxis(3);
        double rightRotation = driverXboxController.getRawAxis(4);
        return rightRotation - leftRotation;
    }

    public Translation2d getCenterOfRotation() {
        double rotX = driverXboxController.getRawAxis(2) * MeasurementConstants.WHEELBASE_M;
        double rotY = driverXboxController.getRawAxis(5) * MeasurementConstants.TRACKWIDTH_M;
        if (rotX * rotY > 0) {
            rotX = -rotX;
            rotY = -rotY;
        }
        rotX *= 0.75;
        rotY *= 0.75;
        Translation2d output = new Translation2d(rotX, rotY);
        return output;
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