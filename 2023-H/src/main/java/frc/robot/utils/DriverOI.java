package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands.ClimbCSGyro;
import frc.robot.commands.DriveCommands.IntakeAlign;
import frc.robot.commands.DriveCommands.LockDrivetrain;
import frc.robot.commands.DriveCommands.RotateToAngle;
import frc.robot.commands.DriveCommands.RotateToAngleWhileDriving;
import frc.robot.commands.DriveCommands.ScoreAlign;
import frc.robot.commands.DriveCommands.SingleSSAlign;
import frc.robot.commands.DriveCommands.StraightenDrivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;

public class DriverOI {
    public static DriverOI instance;

    private final Drivetrain drivetrain;
    private final Claw claw;
    private final Arm arm;
    private final Blinkin blinkin;
    //private final Autonomous autonomous;
    private final Superstructure superstructure;

    private final PS4Controller controller = new PS4Controller(0);

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    private Trigger leftBumperButton, rightBumperButton, muteButton, xButton;

    private boolean returnL3ConeInvertedToPreScore;
    private boolean returnForwardL2L3ScoringPosesToPreScore;

    public enum DPadDirection {
        NONE, FORWARDS, LEFT, RIGHT, BACKWARDS
    };

    public enum DriveSpeedMode {
        NORMAL, SLOW
    };

    private DriveSpeedMode driveSpeedMode;

    public DriverOI() {
        drivetrain = Drivetrain.getInstance();
        claw = Claw.getInstance();
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
        //autonomous = Autonomous.getInstance();
        superstructure = Superstructure.getInstance();

        driveSpeedMode = DriveSpeedMode.NORMAL;

        returnL3ConeInvertedToPreScore = OIConstants.kReturnL3ConeInvertedToPreScore;
        returnForwardL2L3ScoringPosesToPreScore = OIConstants.kReturnForwardL2L3ScoringPosesToPreScore;

        configureController();

    }

    public void controlLoop(){
        if(leftBumperButton.getAsBoolean()){
            if(claw.isEitherSensor()){
                superstructure.requestState(SuperstructureState.EJECTING_GAMEPIECE);
            } else {
                superstructure.requestState(SuperstructureState.CUBE_INTAKE_GROUND);
            }
        } else if(rightBumperButton.getAsBoolean()){
            if(claw.isBothSensors()) {
                superstructure.requestState(SuperstructureState.EJECTING_GAMEPIECE);
            }
            else {
                superstructure.requestState(SuperstructureState.CONE_INTAKE_GROUND);
            }
        } else if(muteButton.getAsBoolean()){
            superstructure.requestState(SuperstructureState.STOWED);
        } else if(xButton.getAsBoolean()){
            superstructure.requestState(SuperstructureState.HP_STATION_INTAKE);
        }
    }

    public void configureController() {

        // Cone intake/eject gamepiece
         leftBumperButton = new JoystickButton(controller, PS4Controller.Button.kL1.value);

        // Cube intake/eject gamepiece
         rightBumperButton = new JoystickButton(controller, PS4Controller.Button.kR1.value);

        // Double substation (human player) cone loading
        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
   
        // Single substation (cone) intake
        xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);

        // Single substation (cube) intake
        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);

        // Set stowed pose
         muteButton = new JoystickButton(controller, 15);

        // Auto-align to score, or to single substation
        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);

        // Lock drivetrain (toggle)
        Trigger rightStickButton = new JoystickButton(controller, PS4Controller.Button.kR3.value);

        // Left stick button, unused
        Trigger leftStickButton = new JoystickButton(controller, PS4Controller.Button.kL3.value);

        // Back button (Touchpad button on front), snaps robot to goal heading
        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);

        // Slow Mode
        // Back button (Option button on front)
        Trigger optionsButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);

        // Share button unused
        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);

        // Reset gyro (resets field oriented drive)
        Trigger ps4Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        ps4Button.onTrue(new InstantCommand(() -> drivetrain.resetGyro()));
    }

    public static DriverOI getInstance() {
        if (instance == null) {
            instance = new DriverOI();
        }
        return instance;
    }

    public void setDriveSpeedMode(DriveSpeedMode mode) {
        driveSpeedMode = mode;
    }

    public void toggleDriveSpeedMode(){
        if(driveSpeedMode.equals(DriveSpeedMode.NORMAL)){
            driveSpeedMode = DriveSpeedMode.SLOW;
        } else {
            driveSpeedMode = DriveSpeedMode.NORMAL;
        }
    }

    public double getForward() {
        double input = controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        if(Math.abs(input) < 0.9){
            input *= 0.7777;
        }
        else{
            input = Math.pow(input, 3);
        }
        return input;
    }

    public double getStrafe() {
        double input = controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
        if(Math.abs(input) < 0.9){
            input *= 0.7777;
        }
        else{
            input = Math.pow(input, 3);
        }
        return input;
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double xSpeed = getForward();
        double ySpeed = getStrafe();

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (DriveConstants.kUseRateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // accelerationcurrentTranslationDir
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
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

            double new_translation_x = next_translation.getX()
                    - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double new_translation_y = next_translation.getY()
                    - (deadband_vector.getY()) / (1 - deadband_vector.getY());

            next_translation = new Translation2d(
                    new_translation_x * getTranslationSpeedCoeff() * DriveConstants.kMaxFloorSpeed,
                    new_translation_y * getTranslationSpeedCoeff() * DriveConstants.kMaxFloorSpeed);

            return next_translation;
        }
    }

    public double getTranslationSpeedCoeff() {
        if (driveSpeedMode == DriveSpeedMode.SLOW) {
            return DriveConstants.kSlowModeTranslationSpeedScale;
        } else {
            return DriveConstants.kNormalModeTranslationSpeedScale;
        }
    }

    public double getRotationSpeedCoeff() {
        if (driveSpeedMode == DriveSpeedMode.SLOW) {
            return DriveConstants.kSlowModeRotationSpeedScale;
        } else {
            return DriveConstants.kNormalModeRotationSpeedScale;
        }
    }

    public Translation2d fromPolar(Rotation2d direction, double magnitude) {
        return new Translation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

    public double getRotation() {
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);

        double combinedRotation;
        if (DriveConstants.kUseRateLimit) {
            combinedRotation = m_rotLimiter.calculate((rightRotation - leftRotation) / 2.0);
        } else {
            combinedRotation = (rightRotation - leftRotation) / 2.0;
        }

        return combinedRotation * getRotationSpeedCoeff() * DriveConstants.kMaxAngularSpeed;
    }

    public Translation2d getCenterOfRotation() {
        double rotX = controller.getRawAxis(2) * DriveConstants.kWheelBase;
        double rotY = controller.getRawAxis(5) * DriveConstants.kTrackWidth;

        if (rotX * rotY > 0) {
            rotX = -rotX;
            rotY = -rotY;
        }
        rotX *= 0.75;
        rotY *= 0.75;
        Translation2d output = new Translation2d(rotX, rotY);
        return output;
    }

    public DPadDirection getDriverDPadInput() {
        switch (controller.getPOV()) {
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

    public Translation2d getCardinalDirection() {
        switch (getDriverDPadInput()) {
            case FORWARDS:
                return new Translation2d(-DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed,
                        0.0);
            case RIGHT:
                return new Translation2d(0.0,
                        DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed);
            case LEFT:
                return new Translation2d(0.0,
                        -DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed);
            case BACKWARDS:
                return new Translation2d(DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed,
                        0.0);
            default:
                return new Translation2d(0.0, 0.0);
        }

    }

    // For testing purposes/ open loop mode.
    public double getArmSpeed() {
        if (Math.abs(controller.getRawAxis(PS4Controller.Axis.kRightY.value)) > 0.01) {
            return -controller.getRawAxis(PS4Controller.Axis.kRightY.value) * 0.6;
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

    public boolean isReturnL3ConeInvertedToPreScore() {
        return returnL3ConeInvertedToPreScore;
    }

    public boolean isReturnForwardL2L3ScoringPosesToPreScore() {
        return returnForwardL2L3ScoringPosesToPreScore;
    }

    public boolean touchpadHeld(){
        return controller.getTouchpad();
    }
}