package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class OperatorTab extends ShuffleboardTabBase {
    private Claw claw = Claw.getInstance();
    private Wrist wrist = Wrist.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Arm arm = Arm.getInstance();
    private GenericEntry mOperatorHasCube;
    private GenericEntry mOperatorHasCone;

    private GenericEntry mClawIsIntaking;
    private GenericEntry mWristIsMoving;
    private GenericEntry mShoulderIsMoving;

    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        mOperatorHasCube = tab
                .add("hasCube", false)
                .withSize(1, 1)
                .getEntry();
        mOperatorHasCone = tab
                .add("hasCone", false)
                .withSize(1, 1)
                .getEntry();
        mClawIsIntaking = tab
                .add("isIntaking", false)
                .withSize(3, 2)
                .getEntry();
        mWristIsMoving = tab
                .add("wristIsMoving", false)
                .withSize(3, 2)
                .getEntry();
        mShoulderIsMoving = tab
                .add("shoulderIsMoving", false)
                .withSize(3, 2)
                .getEntry();
    }

    @Override
    public void update() {
        mOperatorHasCube.setBoolean(claw.hasCube());
        mOperatorHasCone.setBoolean(claw.hasCone());
        mClawIsIntaking.setBoolean(claw.isIntaking());
        mWristIsMoving.setBoolean(wrist.isMoving());
        mShoulderIsMoving.setBoolean(shoulder.isMoving());
    }
}
