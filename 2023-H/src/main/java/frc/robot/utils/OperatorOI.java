package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Claw.ClawState;

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
    private Claw claw;    

    public OperatorOI() {
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        configureController();
    }

    public int getAlignGoalAprilTagID() {
        return alignGoalAprilTagID;
    }

    public AlignGoalColumn getAlignGoalColumn() {
        return alignGoalColumn;
    }

    public void configureController() {
        controller = new PS4Controller(1);

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

        // Arm Poses
        // L1 score
        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new SetLevelOnePose());

        // Request L2 score / transition to pre-score pose
        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        circleButton.onTrue(new ParallelCommandGroup(new SetPreScorePose(), new ConditionalCommand(new InstantCommand(arm::setGoalPoseToLevelTwoCone), new InstantCommand(arm::setGoalPoseToLevelTwoCube), claw::hasCone)));

        // Request L3 score / transition to pre-score pose
        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        triangleButton.onTrue(new ParallelCommandGroup(new SetPreScorePose(), new ConditionalCommand(new InstantCommand(arm::setGoalPoseToLevelThreeCone), new InstantCommand(arm::setGoalPoseToLevelThreeCube), claw::hasCone)));

        // Force score pose
        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        squareButton.onTrue(new InstantCommand(() -> arm.moveToScoringPose()));

        // Stowed pose
        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new SetStowedPose());

        // Home the entire arm subsystem (full system reset)
        Trigger muteButton = new JoystickButton(controller, 15);
        muteButton.onTrue(new SetHomePose());

        // Manual Wrist and Shoulder Override Controls
        Trigger leftTriggerPressedTrigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);
        leftTriggerPressedTrigger.whileTrue(new ManualWristControl());

        Trigger rightTriggerPressedTrigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);
        rightTriggerPressedTrigger.whileTrue(new ManualShoulderControl());

        // Gyro reset
        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        ps5Button.onTrue(new InstantCommand(Drivetrain.getInstance()::resetGyro));

        // toggle outtake full speed or off
        Trigger startButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);
        startButton.onTrue(new InstantCommand(() -> {
            if (claw.getClawSpeed() > Constants.OIConstants.kMaxSpeedThreshold) {
                claw.stopClaw();
            } else {
                claw.setSpeed(1);
            }
        }));

        // toggle intake full reverse speed or off
        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);
        shareButton.onTrue(new InstantCommand(() -> {
            if (claw.getClawSpeed() < -Constants.OIConstants.kMaxSpeedThreshold) {
                claw.stopClaw();
            } else {
                claw.setSpeed(-1);
            }
        }));

        // Game Piece Selection
        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        L1Bumper.onTrue(new InstantCommand(() -> {
            if(bothBumpersHeld()){
                claw.setState(ClawState.EMPTY);
            }
            else{
                claw.setState(ClawState.CONE);
            }
        }));

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        R1Bumper.onTrue(new InstantCommand(() -> {
            if(bothBumpersHeld()){
                claw.setState(ClawState.EMPTY);
            }
            else{
                claw.setState(ClawState.CUBE);
            }
        }));

    }

    private boolean bothBumpersHeld() {
        return controller.getL1Button() && controller.getR1Button();
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
}