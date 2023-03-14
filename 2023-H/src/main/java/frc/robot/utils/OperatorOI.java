package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommands.ManualShoulderControl;
import frc.robot.commands.ArmCommands.ManualWristControl;
import frc.robot.commands.ArmCommands.SetHomePose;
import frc.robot.commands.ArmCommands.SetLevelOnePose;
import frc.robot.commands.ArmCommands.SetLevelThreeConeInvertedPose;
import frc.robot.commands.ArmCommands.SetLevelThreeCubeForwardPose;
import frc.robot.commands.ArmCommands.SetLevelTwoConePose;
import frc.robot.commands.ArmCommands.SetLevelTwoCubePose;
import frc.robot.commands.ArmCommands.SetPreScorePose;
import frc.robot.commands.ArmCommands.SetShoulderHomePose;
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.commands.ArmCommands.SetWristHomePose;
import frc.robot.commands.LimelightCommands.LocalizeWithLL;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.GlobalConstants;
import frc.robot.utils.Constants.OIConstants;

public class OperatorOI {
    public enum AlignGoalColumn {
        kCenter, kLeft, kRight
    }

    private static OperatorOI instance;

    public static OperatorOI getInstance() {
        if (instance == null) {
            instance = new OperatorOI();
        }

        return instance;
    }

    private PS4Controller controller;

    /**
     * the center depending on aliance
     */
    private int alignGoalAprilTagID = DriverStation.getAlliance() == Alliance.Blue ? 7 : 2;
    private AlignGoalColumn alignGoalColumn = AlignGoalColumn.kCenter;

    private Arm arm;
    private Blinkin blinkin;
    private Claw claw;    

    private boolean usePreScorePose;

    public OperatorOI() {
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();

        usePreScorePose = OIConstants.kUsePreScorePose;

        configureController(usePreScorePose);
    }

    public int getAlignGoalAprilTagID() {
        return alignGoalAprilTagID;
    }

    public AlignGoalColumn getAlignGoalColumn() {
        return alignGoalColumn;
    }

    public void configureController(boolean usePreScorePose) {
        controller = new PS4Controller(1);

        // Arm Poses
        // L1 score (will move to this pose regardless of having a gamepiece)
        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelOnePose()));

        /**
         * The following button bindings are for the mode where the robot 'pre-poses' to score.
         * We go to the Pre-Score pose and wait for either operator override to force the final scoring pose,
         * or the driver to call for auto-alignment.
         */
        if(usePreScorePose){
            // Request L2 score / transition to pre-score pose
            // If we're already in a scoring pose, just go there directly.
            // Refuse to do anything if we don't have a game piece yet.
            Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
            circleButton.onTrue(new ConditionalCommand(
                /*
                 * This code runs if we are in an arm scoring pose.
                 */
                // Go directly to the appropriate L2 scoring pose dependent on the game piece.
                new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelTwoCube),
                new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelTwoCubePose()),
                    new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelTwoCone),
                    new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelTwoConePose()),
                        new InstantCommand(() -> {blinkin.failure();}),
                        claw::hasCone),
                    claw::hasCube),
                /**
                 * This code runs if we are NOT in an arm scoring pose already. Typical case: stowed.
                 */
                // If we have a game piece, pre-pose the arm for L2 cube or L2 cone based on intake state.
                new ConditionalCommand(new SequentialCommandGroup(
                    new ConditionalCommand(new InstantCommand(arm::setGoalPoseToLevelTwoCube),
                        new InstantCommand(arm::setGoalPoseToLevelTwoCone),
                        claw::hasCube), new SetPreScorePose()),
                    // If we do not have a game piece, indicate failure.
                    new InstantCommand(() -> {blinkin.failure();}),
                    claw::hasGamepiece),
                arm::isArmScoringPose));

            // Request L3 score / transition to pre-score pose
            // If we're already in a scoring pose, just go there directly.
            // Refuse to do anything if we don't have a game piece yet.
            Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
            triangleButton.onTrue(new ConditionalCommand(
                /*
                 * This code runs if we are in an arm scoring pose.
                 */
                // Go directly to the appropriate L3 scoring pose dependent on the game piece.
                new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelThreeCubeForward),
                new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelThreeCubeForwardPose()),
                    new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelThreeConeInverted),
                    new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelThreeConeInvertedPose()),
                        new InstantCommand(() -> {blinkin.failure();}),
                        claw::hasCone),
                    claw::hasCube),
                /**
                 * This code runs if we are NOT in an arm scoring pose already. Typical case: stowed.
                 */
                // If we have a game piece, pre-pose the arm for L3 cube or L3 cone based on intake state.
                new ConditionalCommand(new SequentialCommandGroup(
                    new ConditionalCommand(new InstantCommand(arm::setGoalPoseToLevelThreeCubeForward),
                        new InstantCommand(arm::setGoalPoseToLevelThreeConeInverted), 
                        claw::hasCube), new SetPreScorePose()),
                    // If we do not have a game piece, indicate failure.
                    new InstantCommand(() -> {blinkin.failure();}),
                    claw::hasGamepiece),
                arm::isArmScoringPose));

            // Force score pose from pre-score pose
            Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
            squareButton.onTrue(new InstantCommand(() -> arm.moveToScoringPose()));
        }
        /**
         * The following button bindings are for the mode where the robot goes directly to each scoring pose.
         * Refuse to do anything if we don't have a game piece yet.
         */ 
        else{
            // L2 scoring pose (provided we have a gamepiece)
            Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
            circleButton.onTrue(new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelTwoCube),
                new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelTwoCubePose()),
                new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelTwoCone),
                    new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelTwoConePose()),
                        new InstantCommand(() -> {blinkin.failure();}),
                        claw::hasCone),
                    claw::hasCube));

            // L3 scoring pose (provided we have a gamepiece)
            Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
            triangleButton.onTrue(new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelThreeCubeForward),
                new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelThreeCubeForwardPose()),
                new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelThreeConeInverted),
                    new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelThreeConeInvertedPose()),
                        new InstantCommand(() -> {blinkin.failure();}),
                        claw::hasCone),
                    claw::hasCube));


            // Square button forces the robot to look at odometry updates.
            Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
            squareButton.onTrue(new LocalizeWithLL());
        }

        // Stowed pose
        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new SetStowedPose());

        // Mute ONLY homes the wrist without moving the shoulder
        // Mute + d-pad (down) homes the entire arm subsystem (full system reset, a bit slower)
        Trigger muteButton = new JoystickButton(controller, 15);
        muteButton.onTrue(new ConditionalCommand(new SetHomePose(), new SetWristHomePose(), this::dPadDownHeld));

        // Manual Wrist and Shoulder Override Controls
        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);
        L2Trigger.whileTrue(new ConditionalCommand(new ManualWristControl(), new InstantCommand(), this::isLeftStickActive));

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);
        R2Trigger.whileTrue(new ConditionalCommand(new ManualShoulderControl(), new InstantCommand(), this::isRightStickActive));

        // Gyro reset
        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        ps5Button.onTrue(new InstantCommand(Drivetrain.getInstance()::resetGyro));

        // Toggle outtake at varying speeds depending on trigger modification
        // Default behavior is slow
        // Medium and fast speeds can be accessed with left/right trigger modification, respectively
        Trigger startButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);
        startButton.onTrue(new InstantCommand(() -> {
            if (leftTriggerHeld()) {
                claw.setSpeed(0.1);
            } else if(rightTriggerHeld()){
                claw.setSpeed(1.0);
            } else {
                claw.setSpeed(0.05);
            }
        }));

        // Toggle intake at varying speeds depending on trigger modification
        // Default behavior is slow
        // Medium and fast speeds can be accessed with left/right trigger modification, respectively
        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);
        shareButton.whileTrue(new InstantCommand(() -> {
            if (leftTriggerHeld()) {
                claw.setSpeed(-0.1);
            } else if(rightTriggerHeld()){
                claw.setSpeed(-1.0);
            } else {
                claw.setSpeed(-0.05);
            }
        }));

        // Game piece selection / LED indication requests to human player
        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        L1Bumper.onTrue(new InstantCommand(() -> {
            if(bothBumpersHeld()){
                blinkin.returnToRobotState();
            }
            else{
                blinkin.intakingCone();
            }
        }));

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        R1Bumper.onTrue(new InstantCommand(() -> {
            if(bothBumpersHeld()){
                blinkin.returnToRobotState();
            }
            else{
                blinkin.intakingCube();
            }
        }));

        // Column Selection
        Trigger dpadUpTrigger = new Trigger(() -> controller.getPOV() == 0);
        dpadUpTrigger.onTrue(new InstantCommand(() -> {
            if (bothBumpersHeld()) {
                alignGoalAprilTagID = DriverStation.getAlliance() == Alliance.Blue ? 7 : 2;
            } else {
                alignGoalColumn = AlignGoalColumn.kCenter;
            }
        }));

        Trigger dpadLeftTrigger = new Trigger(() -> controller.getPOV() == 270);
        dpadLeftTrigger.onTrue(new InstantCommand(() -> {
            if (bothBumpersHeld()) {
                alignGoalAprilTagID = DriverStation.getAlliance() == Alliance.Blue ? 6 : 1;
            } else {
                alignGoalColumn = AlignGoalColumn.kLeft;
            }
        }));

        Trigger dpadRightTrigger = new Trigger(() -> controller.getPOV() == 90);
        dpadRightTrigger.onTrue(new InstantCommand(() -> {
            if (bothBumpersHeld()) {
                alignGoalAprilTagID = DriverStation.getAlliance() == Alliance.Blue ? 8 : 3;
            } else {
                alignGoalColumn = AlignGoalColumn.kRight;
            }
        }));

        Trigger dpadDownTrigger = new Trigger(() -> controller.getPOV() == 180);

    }

    private boolean bothBumpersHeld() {
        return controller.getL1Button() && controller.getR1Button();
    }

    private boolean bothTriggersHeld() {
        return leftTriggerHeld() & rightTriggerHeld();
    }

    private boolean leftTriggerHeld(){
        return controller.getL2Button();
    }

    private boolean onlyLeftTriggerHeld(){
        return leftTriggerHeld() && !rightTriggerHeld();
    }

    private boolean rightTriggerHeld(){
        return controller.getR2Button();
    }

    private boolean onlyRightTriggerHeld(){
        return !leftTriggerHeld() && rightTriggerHeld();
    }

    private boolean onlyOneTriggerHeld() {
        return leftTriggerHeld() ^ rightTriggerHeld();
    }

    private boolean dPadDownHeld(){
        return controller.getPOV() == 180;
    }

    public boolean isLeftStickActive(){
        return controller.getRawAxis(PS4Controller.Axis.kLeftY.value) != 0;
    }

    public boolean isRightStickActive(){
        return controller.getRawAxis(PS4Controller.Axis.kRightY.value) != 0;
    }

    public double getShoulderPIDOffset() {
        double rawAxis = controller.getRawAxis(PS4Controller.Axis.kRightY.value);
        if (Math.abs(rawAxis) < Constants.OIConstants.kDrivingDeadband) {
            return 0;
        }
        return -1*rawAxis/4;
    }

    public double getWristPIDOffset() {
        double rawAxis = controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        if (Math.abs(rawAxis) < Constants.OIConstants.kDrivingDeadband) {
            return 0;

        }
        return -1*rawAxis;
    }

    public boolean isUsePreScorePose() {
        return usePreScorePose;
    }

    // Only update the boolean for using the pre-score pose if it is a state change
    // This is especially important since this requires configuring the controller mapping
    // for the operator, which should be done infrequently/minimally.
    public void setUsePreScorePose(boolean usePreScorePose) {
        if(this.usePreScorePose != usePreScorePose){
            this.usePreScorePose = usePreScorePose;
            configureController(usePreScorePose);
        }
    }

}