package frc.robot.utils;

import javax.sql.rowset.spi.TransactionalWriter;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.utils.Constants.MeasurementConstants;
import frc.robot.utils.Constants.OIConstants;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import frc.robot.subsystems.Claw;

public class OI {
    public static OI instance;
    private Drivetrain drivetrain;
    private Claw intake;
    // private Joystick leftJoystick = new Joystick(0);
    // private Joystick rightJoystick = new Joystick(1);

    private PS4Controller driverPs4Controller = new PS4Controller(0); // this is no longer just a random number

    private final SlewRateLimiter slewX = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter slewY = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(DriveConstants.kRotationSlewRate);
    private double rotation;

    public OI() {
        drivetrain = Drivetrain.getInstance();
        // Resets the gyro
        // Trigger resetGyroButton = new JoystickButton(driverPs4Controller, 1);
        // resetGyroButton.whileTrue(new InstantCommand(() -> drivetrain.resetGyro()));

        // Snap to angle
        // Trigger snapToAngle = new JoystickButton(driverPs4Controller, 5);
        // snapToAngle.whileTrue(new SnapToAngle());

        Trigger xButton = new JoystickButton(driverPs4Controller, PS4Controller.Button.kCross.value);
        // TODO: ejects game piece
        xButton.onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("Circle Pressed", true)));
        xButton.onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Circle Pressed", false)));

        Trigger circleButton = new JoystickButton(driverPs4Controller, PS4Controller.Button.kCircle.value);
        // TODO: runs level 2 scoring pose function

        Trigger squareButton = new JoystickButton(driverPs4Controller, PS4Controller.Button.kSquare.value);
        // TODO: runs human station intake pose function

        Trigger triangleButton = new JoystickButton(driverPs4Controller, PS4Controller.Button.kTriangle.value);
        // TODO: runs level 3 scoring pose function

        Trigger leftBumperButton = new JoystickButton(driverPs4Controller, PS4Controller.Button.kL1.value);
        // TODO: changes to slow mode

        Trigger rightBumperButton = new JoystickButton(driverPs4Controller, PS4Controller.Button.kR1.value);
        // TODO: runs auto-aligner/driver assist

        Trigger leftStickButton = new JoystickButton(driverPs4Controller, PS4Controller.Button.kL3.value);
        // TODO: runs X lock (safe pose)

        Trigger rightStickButton = new JoystickButton(driverPs4Controller, PS4Controller.Button.kR3.value);
        // TODO: runs rotations lock

        Trigger backButton = new JoystickButton(driverPs4Controller, PS4Controller.Button.kShare.value);
        // TODO: runs cone floor intake/level 1 scoring pose

        Trigger startButton = new JoystickButton(driverPs4Controller, PS4Controller.Button.kOptions.value);
        // TODO: runs cube floor intake/level 1 scoring pose

        Trigger ps4Button = new JoystickButton(driverPs4Controller, PS4Controller.Button.kPS.value);
        ps4Button.onTrue(new InstantCommand(() -> drivetrain.resetGyro()));
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public double getForward() {
        return driverPs4Controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
    }

    public double getStrafe() {
        return driverPs4Controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
    }

    public Translation2d getTranslation2d() { // TODO: Cardinal directions
        Translation2d position = new Translation2d(
                slewX.calculate(-inputTransform(getForward())) * DriveConstants.kMaxSpeedMetersPerSecond,
                slewY.calculate(-inputTransform(getStrafe())) * DriveConstants.kMaxSpeedMetersPerSecond);

        return position;
    }

    public double getRotation() {
        double leftRotation = driverPs4Controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double rightRotation = driverPs4Controller.getRawAxis(PS4Controller.Axis.kR2.value);
        return rightRotation - leftRotation;
    }

    public Translation2d getCenterOfRotation() {
        double rotX = driverPs4Controller.getRawAxis(2) * MeasurementConstants.WHEELBASE_M;
        double rotY = driverPs4Controller.getRawAxis(5) * MeasurementConstants.TRACKWIDTH_M;
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