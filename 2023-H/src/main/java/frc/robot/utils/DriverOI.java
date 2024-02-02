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
import frc.robot.commands.ArmCommands.SetDoubleSSConePose;
import frc.robot.commands.ArmCommands.SetExtendedFloorConePose;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubePose;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubePoseOld;
import frc.robot.commands.ArmCommands.SetLevelTwoConeShootL1Pose;
import frc.robot.commands.ArmCommands.SetLevelTwoConeStowedPose;
import frc.robot.commands.ArmCommands.SetPreScorePose;
import frc.robot.commands.ArmCommands.SetPreScorePoseL3Return;
import frc.robot.commands.ArmCommands.SetSingleSSConePose;
import frc.robot.commands.ArmCommands.SetSingleSSCubePose;
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.commands.ArmCommands.SetTransitoryPoseL3Return;
import frc.robot.commands.ClawCommands.NormalizeConeAfterIntake;
import frc.robot.commands.ClawCommands.BackwardsConeShot;
import frc.robot.commands.ClawCommands.EjectGamepiece;
import frc.robot.commands.ClawCommands.IntakeFloorCone;
import frc.robot.commands.ClawCommands.IntakeConeSingleSS;
import frc.robot.commands.ClawCommands.IntakeFloorCube;
import frc.robot.commands.ClawCommands.IntakeCubeSingleSS;
import frc.robot.commands.DriveCommands.SourceSideAlign;
import frc.robot.commands.DriveCommands.SpeakerSquareThenAlign;
import frc.robot.commands.DriveCommands.ApriltagBotPoseAlign;
import frc.robot.commands.DriveCommands.ClimbCSGyro;
import frc.robot.commands.DriveCommands.FollowNote;
import frc.robot.commands.DriveCommands.IntakeAlign;
import frc.robot.commands.DriveCommands.LockDrivetrain;
import frc.robot.commands.DriveCommands.RotateToAngle;
import frc.robot.commands.DriveCommands.RotateToAngleWhileDriving;
import frc.robot.commands.DriveCommands.ScoreAlign;
import frc.robot.commands.DriveCommands.SingleSSAlign;
import frc.robot.commands.DriveCommands.StraightenDrivetrain;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;

public class DriverOI {
    public static DriverOI instance;

    private final Drivetrain drivetrain;
    private final Claw claw;
    private final Arm arm;
    private final Blinkin blinkin;
    // private final Autonomous autonomous;

    private final PS4Controller controller = new PS4Controller(0);

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

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
        // autonomous = Autonomous.getInstance();

        driveSpeedMode = DriveSpeedMode.NORMAL;

        returnL3ConeInvertedToPreScore = OIConstants.kReturnL3ConeInvertedToPreScore;
        returnForwardL2L3ScoringPosesToPreScore = OIConstants.kReturnForwardL2L3ScoringPosesToPreScore;

        configureController();

    }

    public void configureController() {

        // Cone intake/eject gamepiece
        Trigger leftBumperButton = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        leftBumperButton.onTrue(new ConditionalCommand(
            /*
             * If we're in a valid ejection pose, eject, and then if we're L3 inverted, un-invert.
             * NOTE: If we are not yet in a scoring pose and a goal scoring pose has been set, this moves immediately
             * to the goal pose, bypassing auto-align.
             * When we un-invert, either go to the pre-score score pose or stow, depending on the loaded setting.
             * Or, if we are L1 pose, stow automatically.
             * Otherwise do nothing.
             */
            new SequentialCommandGroup(new InstantCommand(() -> arm.moveToScoringPose()), new EjectGamepiece(),
                new ConditionalCommand(
                    // Either return to pre-score or stow from L3 cone inverted
                    new ConditionalCommand(new SetPreScorePoseL3Return(),
                        new SequentialCommandGroup(new SetTransitoryPoseL3Return(), new SetStowedPose()),
                        this::isReturnL3ConeInvertedToPreScore),
                        // If we're in L1, just stow.
                    new ConditionalCommand(new SetStowedPose(),
                        // If we're not in L3 cone inverted or L1, check if we're in any other forward scoring pose.
                        // Depending on the settings, either go to pre-score or do nothing
                        new ConditionalCommand(new ConditionalCommand(new SetPreScorePose(), new InstantCommand(), arm::isL2L3ForwardScoringPose),
                            new InstantCommand(),
                            this::isReturnForwardL2L3ScoringPosesToPreScore),
                        arm::isL1Pose),
                    arm::isInvertedL3)),
            /*
             * If we are not in a valid ejection pose, then we should do floor cone intake, and stow when we have
             * a gamepiece.
             */
            new SequentialCommandGroup(new ParallelRaceGroup(new SetExtendedFloorConePose(), new IntakeFloorCone()),
                new ParallelCommandGroup(new SetStowedPose(), new ConditionalCommand(new NormalizeConeAfterIntake(), new InstantCommand(), claw::hasCone))),
            arm::isValidEjectPose));

        // Cube intake/eject gamepiece
        Trigger rightBumperButton = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        rightBumperButton.onTrue(new ConditionalCommand(
            /*
             * If we're in a valid ejection pose, eject, and then if we're L3 inverted, un-invert and stow.
             * NOTE: If we are not yet in a scoring pose and a goal scoring pose has been set, this moves immediately
             * to the goal pose, bypassing auto-align.
             * When we un-invert, either go to the pre-score score pose or stow, depending on the loaded setting.
             * Or, if we are L1 pose, stow automatically.
             * Otherwise do nothing.
             */
            new SequentialCommandGroup(new InstantCommand(() -> arm.moveToScoringPose()), new EjectGamepiece(),
                new ConditionalCommand(
                    // Either return to pre-score or stow from L3 cone inverted
                    new ConditionalCommand(new SetPreScorePoseL3Return(),
                        new SequentialCommandGroup(new SetTransitoryPoseL3Return(), new SetStowedPose()),
                        this::isReturnL3ConeInvertedToPreScore),
                        // If we're in L1, just stow.
                    new ConditionalCommand(new SetStowedPose(),
                        // If we're not in L3 cone inverted or L1, check if we're in any other forward scoring pose.
                        // Depending on the settings, either go to pre-score or do nothing
                        new ConditionalCommand(new ConditionalCommand(new SetPreScorePose(), new InstantCommand(), arm::isL2L3ForwardScoringPose),
                            new InstantCommand(),
                            this::isReturnForwardL2L3ScoringPosesToPreScore),
                        arm::isL1Pose),
                    arm::isInvertedL3)),
            /*
             * If we are not in a valid ejection pose, then we should do floor cube intake, and stow when we have
             * a gamepiece.
             */
            new SequentialCommandGroup(new ConditionalCommand(new ParallelCommandGroup(new SetExtendedFloorCubePose(), new IntakeFloorCube()), new ParallelCommandGroup(new SetExtendedFloorCubePoseOld(), new IntakeFloorCube()), arm::canNewIntake), new ParallelCommandGroup(new SetStowedPose(), new ConditionalCommand(new NormalizeConeAfterIntake(), new InstantCommand(), claw::hasCone))),
            arm::isValidEjectPose));

        // Double substation (human player) cone loading
        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        // triangleButton.onTrue(new ParallelCommandGroup(new SetDoubleSSConePose(), new IntakeFloorCone()));
        triangleButton.toggleOnTrue(new SourceSideAlign()); 
   
        // Single substation (cone) intake
        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        // xButton.onTrue(new SequentialCommandGroup(new ParallelCommandGroup(new SetSingleSSConePose(), new IntakeConeSingleSS()), new SetStowedPose()));
        xButton.whileTrue(new FollowNote());
        
        // Single substation (cube) intake
        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        squareButton.whileTrue(new SpeakerSquareThenAlign());
        // squareButton.onTrue(new SequentialCommandGroup(new ParallelCommandGroup(new SetSingleSSCubePose(), new IntakeCubeSingleSS()), new SetStowedPose()));

        // Set stowed pose
        Trigger muteButton = new JoystickButton(controller, 15);
        muteButton.onTrue(new SetStowedPose());

        // Auto-align to score, or to single substation
        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        circleButton.whileTrue(
            new ConditionalCommand(
                /*
                 * If we have a gamepiece, perform auto-align to score.
                 * Also: transition from pre-score pose to scoring pose if needed.
                 */
                new ConditionalCommand(new ParallelCommandGroup(new ScoreAlign(),
                    new ConditionalCommand(new InstantCommand(() -> {arm.moveToScoringPose();}), new InstantCommand(), arm::isPreScorePose)),
                new InstantCommand(() -> {blinkin.failure();}),
                arm::isAutoAlignValid),
                /*
                 * If we do not have a gamepiece, perform auto-align to the single substation,
                 * provided we have been commanded to one of those poses. Otherwise refuse to do anything/failure mode. 
                 */
                new ConditionalCommand(new SingleSSAlign(), new InstantCommand(() -> {blinkin.failure();}), arm::isSingleSSPose),
            claw::hasGamepiece));

        // Lock drivetrain (toggle)
        Trigger rightStickButton = new JoystickButton(controller, PS4Controller.Button.kR3.value);
        rightStickButton.toggleOnTrue(new LockDrivetrain());

        // Left stick button, unused
        Trigger leftStickButton = new JoystickButton(controller, PS4Controller.Button.kL3.value);

        // Back button (Touchpad button on front), snaps robot to goal heading
        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new ConditionalCommand(new InstantCommand(),
            new ConditionalCommand(new RotateToAngleWhileDriving(180), new RotateToAngleWhileDriving(0),
                drivetrain::getFlipped),
            arm::isUnsafeSnapPose));

        // Slow Mode
        // Back button (Option button on front)
        Trigger optionsButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);
        optionsButton.onTrue(new InstantCommand(() -> setDriveSpeedMode(DriveSpeedMode.SLOW))).onFalse(new InstantCommand(() -> setDriveSpeedMode(DriveSpeedMode.NORMAL)));

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