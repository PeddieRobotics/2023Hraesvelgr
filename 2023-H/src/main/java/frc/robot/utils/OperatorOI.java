package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

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

    public OperatorOI() {
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

        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new InstantCommand(() -> {
            // Set to eject
        }));

        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        circleButton.onTrue(new InstantCommand(() -> {
            // Set armstate to l2scoring
            // should check for cube or cone
        }));

        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        squareButton.onTrue(new InstantCommand(() -> {
            // Set armstate to humanstationintake
        }));

        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        triangleButton.onTrue(new InstantCommand(() -> {
            // Set armstate to l3scoring
            // should check for cone or cube
        }));

        Trigger leftTriggerPressedTrigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);
        leftTriggerPressedTrigger.whileTrue(new CommandBase() {
            @Override
            public void execute() {
                Shoulder.getInstance().setPosition(Shoulder.getInstance().getPosition() + getShoulderPIDOffset());
                Wrist.getInstance().setPosition(Wrist.getInstance().getPosition() + getWristPIDOffset());
            }
        });

        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        ps5Button.onTrue(new InstantCommand(Drivetrain.getInstance()::resetGyro));

        // Trigger startButton = new JoystickButton(controller,
        // PS4Controller.Button.kOptions.value);
        // // toggle intake full speed or off

        // Trigger backButton = new JoystickButton(controller,
        // PS4Controller.Button.kShare.value);
        // // toggle intake full reverse speed or off
    }

    private boolean bothBumpersHeld() {
        return controller.getL1Button() && controller.getR1Button();
    }

    private double getShoulderPIDOffset() {
        double rawAxis = controller.getLeftY();
        if (rawAxis < Constants.OIConstants.kDrivingDeadband && rawAxis > -Constants.OIConstants.kDrivingDeadband) {
            return 0;
        }

        // TODO: make it so it overrides the driver input
        return Constants.OIConstants.kMaxDeltaShoulderAnglePerSecond * 0.02 * -rawAxis;
    }

    private double getWristPIDOffset() {
        double rawAxis = controller.getRightY();
        if (rawAxis < Constants.OIConstants.kDrivingDeadband && rawAxis > -Constants.OIConstants.kDrivingDeadband) {
            return 0;
        }

        // TODO: make it so it overrides the driver input
        return Constants.OIConstants.kMaxDeltaWristAnglePerSecond * 0.02 * -rawAxis;
    }
}