package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.commands.ClawCommands.OperatorEjectGamepiece;
import frc.robot.commands.ClawCommands.OperatorIntakeGamepiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Drivetrain;
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
    private int alignGoalAprilTagID = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 2;
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
                new SequentialCommandGroup(
                    new ConditionalCommand(new InstantCommand(arm::setGoalPoseToLevelTwoCube),
                        new InstantCommand(arm::setGoalPoseToLevelTwoCone),
                        claw::hasCube), new SetPreScorePose()),
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
                new SequentialCommandGroup(
                    new ConditionalCommand(new InstantCommand(arm::setGoalPoseToLevelThreeCubeForward),
                        new ConditionalCommand(new InstantCommand(arm::setGoalPoseToLevelThreeConeForward),
                            new InstantCommand(arm::setGoalPoseToLevelThreeConeInverted), this::dPadDownHeld), 
                        claw::hasCube), new SetPreScorePose()),
                arm::isArmScoringPose));
;
            // Force score pose from pre-score pose
            Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
            squareButton.onTrue(new InstantCommand(() -> arm.moveToScoringPose()));
        }
        /**
         * The following button bindings are for the mode where the robot goes directly to each scoring pose.
         * Refuse to do anything if we don't have a game piece yet.
         */ 
        else{
            // L2 scoring pose
            Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
            circleButton.onTrue(new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelTwoCube),
                new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelTwoCubePose()),
                new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelTwoCone),
                    new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelTwoConePose()),
                        new InstantCommand(() -> {blinkin.failure();}),
                        claw::hasCone),
                    claw::hasCube));

            // L3 scoring pose - does not include L3 cone forward right now.
            Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
            triangleButton.onTrue(new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelThreeCubeForward),
                new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelThreeCubeForwardPose()),
                new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelThreeConeInverted),
                    new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelThreeConeInvertedPose()),
                        new InstantCommand(() -> {blinkin.failure();}),
                        claw::hasCone),
                    claw::hasCube));
            
            // triangleButton.onTrue(new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelThreeCubeForward),
            //     new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelThreeCubeForwardPose()),
            //     new ConditionalCommand(new SequentialCommandGroup(new InstantCommand(arm::setGoalPoseToLevelThreeConeForward),
            //         new InstantCommand(() -> {claw.prepareLimelightForScoring();}), new SetLevelThreeConeForwardPose()),
            //             new InstantCommand(() -> {blinkin.failure();}),
            //             claw::hasCone),
            //         claw::hasCube));


            // Square button forces the robot to look at odometry updates.
            Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
            //squareButton.whileTrue(new LocalizeWithLL());
        }

        // Stowed pose
        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new SetStowedPose());

        // Mute homes the entire arm subsystem, both wrist and shoulder.
        Trigger muteButton = new JoystickButton(controller, 15);
        muteButton.onTrue(new SetHomePose());

        // Manual Wrist and Shoulder Override Controls
        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);
        L2Trigger.whileTrue(new ManualWristControl());

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);
        R2Trigger.whileTrue(new ManualShoulderControl());

        // Gyro reset
        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        ps5Button.onTrue(new InstantCommand(Drivetrain.getInstance()::resetGyro));

        // Press and hold for outtaking slow (gamepiece adjustment), with down arrow this becomes full speed.
        Trigger startButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);
        startButton.whileTrue(new OperatorEjectGamepiece());

        // Press and hold for intaking slow (gamepiece adjustment), with down arrow this becomes full speed.
        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);
        shareButton.whileTrue(new OperatorIntakeGamepiece());

        // Game piece selection / LED indication requests to human player
        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        L1Bumper.onTrue(new InstantCommand(() -> {
            if(bothBumpersHeld()){
                if(dPadDownHeld()){
                    claw.setState(ClawState.EMPTY);
                }

                blinkin.returnToRobotState();
                claw.setGamepieceOperatorOverride(false);
            }
            else if(dPadDownHeld()){
                claw.setGamepieceOperatorOverride(true);
                claw.setState(ClawState.CONE);
                blinkin.returnToRobotState();
            }
            else{
                blinkin.intakingCone();
            }
        }));

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        R1Bumper.onTrue(new InstantCommand(() -> {
            if(bothBumpersHeld()){
                if(dPadDownHeld()){
                    claw.setState(ClawState.EMPTY);
                }

                blinkin.returnToRobotState();
                claw.setGamepieceOperatorOverride(false);
            }
            else if(dPadDownHeld()){
                claw.setGamepieceOperatorOverride(true);
                claw.setState(ClawState.CUBE);
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
                alignGoalAprilTagID = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 2;
            } else {
                alignGoalColumn = AlignGoalColumn.kCenter;
            }
        }));

        Trigger dpadLeftTrigger = new Trigger(() -> controller.getPOV() == 270);
        dpadLeftTrigger.onTrue(new InstantCommand(() -> {
            if (bothBumpersHeld()) {
                alignGoalAprilTagID = DriverStation.getAlliance().get() == Alliance.Blue ? 6 : 1;
            } else {
                alignGoalColumn = AlignGoalColumn.kLeft;
            }
        }));

        Trigger dpadRightTrigger = new Trigger(() -> controller.getPOV() == 90);
        dpadRightTrigger.onTrue(new InstantCommand(() -> {
            if (bothBumpersHeld()) {
                alignGoalAprilTagID = DriverStation.getAlliance().get() == Alliance.Blue ? 8 : 3;
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

    public boolean dPadDownHeld(){
        return controller.getPOV() == 180;
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