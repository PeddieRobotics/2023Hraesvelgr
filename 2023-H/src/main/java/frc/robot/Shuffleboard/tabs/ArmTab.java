package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.Constants;

public class ArmTab extends ShuffleboardTabBase {
    private Arm arm = Arm.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Wrist wrist = Wrist.getInstance();
    private Claw claw = Claw.getInstance();
    private GenericEntry mOperatorHasCube;
    private GenericEntry mOperatorHasCone;

    private GenericEntry mWristSpeed;
    private GenericEntry mWristPos;
    private GenericEntry mWristIsMoving;

    private GenericEntry mClawSpeed;
    private GenericEntry mClawIsIntaking;

    private GenericEntry mShoulderSpeed;
    private GenericEntry mShoulderPos;
    private GenericEntry mShoulderIsMoving;

    private GenericEntry mSetClawSpeed;
    private GenericEntry mSetShoulderPos;
    private GenericEntry mSetWristPos;

    public void createEntries() {
        tab = Shuffleboard.getTab("Arm");

        mOperatorHasCube = tab
                .add("hasCube", false)
                .getEntry();
        mOperatorHasCone = tab
                .add("hasCone", false)
                .getEntry();
        mWristSpeed = tab
                .add("wristSpeed", 0.0)
                .getEntry();
        mShoulderSpeed = tab
                .add("shoulderSpeed", 0.0)
                .getEntry();
        mClawSpeed = tab
                .add("clawSpeed", 0.0)
                .withSize(3, 2)
                .getEntry();
        mShoulderPos = tab
                .add("shoulderPosition", 0.0)
                .getEntry();
        mWristPos = tab
                .add("wristPosition", 0.0)
                .getEntry();
        mShoulderIsMoving = tab
                .add("shoulderIsMoving", false)
                .getEntry();
        mWristIsMoving = tab
                .add("wristisMoving", false)
                .getEntry();
        mSetShoulderPos = tab
                .add("setShoulderPosition", 0.0)
                .getEntry();
        mSetWristPos = tab
                .add("setWristPosition", 0.0)
                .getEntry();
        mSetClawSpeed = tab
                .add("setClawSpeed", 0.0)
                .getEntry();
        mClawIsIntaking = tab
                .add("clawIsIntaking", false)
                .getEntry();

        tab
                .add("Shoulder kS", Constants.ShoulderConstants.kSVolts)
                .getEntry();
        tab
                .add("Shoulder kG", Constants.ShoulderConstants.kGVolts)
                .getEntry();
        tab
                .add("Shoulder kV", Constants.ShoulderConstants.kVVoltSecondPerRad)
                .getEntry();
        tab
                .add("Shoulder kA", Constants.ShoulderConstants.kAVoltSecondSquaredPerRad)
                .getEntry();
        tab
                .add("Shoulder P", Constants.ShoulderConstants.kP)
                .getEntry();
        tab
                .add("Shoulder I", Constants.ShoulderConstants.kI)
                .getEntry();
        tab
                .add("Shoulder D", Constants.ShoulderConstants.kD)
                .getEntry();
        tab
                .add("Shoulder FF", Constants.ShoulderConstants.kFF)
                .getEntry();

        tab
                .add("Wrist kS", Constants.ShoulderConstants.kSVolts)
                .getEntry();
        tab
                .add("Wrist kG", Constants.ShoulderConstants.kGVolts)
                .getEntry();
        tab
                .add("Wrist kV", Constants.ShoulderConstants.kVVoltSecondPerRad)
                .getEntry();
        tab
                .add("Wrist kA", Constants.ShoulderConstants.kAVoltSecondSquaredPerRad)
                .getEntry();
        tab
                .add("Wrist P", Constants.ShoulderConstants.kP)
                .getEntry();
        tab
                .add("Wrist I", Constants.ShoulderConstants.kI)
                .getEntry();
        tab
                .add("Wrist D", Constants.ShoulderConstants.kD)
                .getEntry();
        tab
                .add("Wrist FF", Constants.ShoulderConstants.kFF)
                .getEntry();
    }

    @Override
    public void update() {
        mOperatorHasCube.setBoolean(claw.hasCube());
        mOperatorHasCone.setBoolean(claw.hasCone());

        mShoulderSpeed.setDouble(shoulder.getSpeed());
        mShoulderPos.setDouble(shoulder.getPosition());
        mShoulderIsMoving.setBoolean(shoulder.isMoving());

        mClawSpeed.setDouble(claw.getClawSpeed());
        mClawIsIntaking.setBoolean(claw.isIntaking());

        mWristSpeed.setDouble(wrist.getSpeed());
        mWristPos.setDouble(wrist.getPosition());
        mWristIsMoving.setBoolean(wrist.isMoving());

        arm.setShoulderPosition(mSetShoulderPos.getDouble(0.0));
        arm.setWristPosition(mSetWristPos.getDouble(0.0));
        claw.setSpeed(mSetClawSpeed.getDouble(0.0));
    }
}
