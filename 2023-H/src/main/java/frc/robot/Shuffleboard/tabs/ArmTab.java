package frc.robot.Shuffleboard.tabs;

import javax.swing.text.TabExpander;

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
    private GenericEntry mWristAngle;
    private GenericEntry mWristIsMoving;
    private GenericEntry mWristCurrent;
    private GenericEntry mWristTemp;
    private GenericEntry mWristArbitraryFF;

    private GenericEntry mClawSpeed;
    private GenericEntry mClawCurrent;
    private GenericEntry mClawIsIntaking;

    private GenericEntry mShoulderSpeed;
    private GenericEntry mShoulderAngle;
    private GenericEntry mShoulderIsMoving;
    private GenericEntry mShoulderCurrent;
    private GenericEntry mShoulderTemp;
    private GenericEntry mShoulderArbitraryFF;

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
                .add("Wrist Speed", 0.0)
                .getEntry();
        mWristAngle = tab
                .add("Wrist Pos", 0.0)
                .getEntry();
        mWristIsMoving = tab
                .add("Wrist isMoving", false)
                .getEntry();
        mWristCurrent = tab
                .add("Wrist Current", 0.0)
                .getEntry();
        mWristTemp = tab
                .add("Wrist Temp", 0.0)
                .getEntry();
        mWristArbitraryFF = tab
                .add("Wrist Arbitrary FF", 0.0)
                .getEntry();

        mShoulderSpeed = tab
                .add("Shoulder Speed", 0.0)
                .getEntry();
        mShoulderAngle = tab
                .add("Shoulder Pos", 0.0)
                .getEntry();
        mShoulderIsMoving = tab
                .add("Shoulder isMoving", false)
                .getEntry();
        mShoulderCurrent = tab
                .add("Shoulder Current", 0.0)
                .getEntry();
        mShoulderTemp = tab
                .add("Shoulder Temp", 0.0)
                .getEntry();
        mShoulderArbitraryFF = tab
                .add("Shoulder FF", 0.0)
                .getEntry();

        mClawSpeed = tab
                .add("Claw Speed", 0.0)
                .getEntry();
        mClawCurrent = tab
                .add("Claw Current", 0.0)
                .getEntry();
        mClawIsIntaking = tab
                .add("Claw isIntaking", false)
                .getEntry();

        mSetShoulderPos = tab
                .add("setShoulderPos", 0.0)
                .getEntry();
        mSetWristPos = tab
                .add("setWristPos", 0.0)
                .getEntry();
        mSetClawSpeed = tab
                .add("setClawSpeed", 0.0)
                .getEntry();

        // tab
        // .add("Shoulder kS", Constants.ShoulderConstants.kSVolts)
        // .getEntry();
        // tab
        // .add("Shoulder kG", Constants.ShoulderConstants.kGVolts)
        // .getEntry();
        // tab
        // .add("Shoulder kV", Constants.ShoulderConstants.kVVoltSecondPerRad)
        // .getEntry();
        // tab
        // .add("Shoulder kA", Constants.ShoulderConstants.kAVoltSecondSquaredPerRad)
        // .getEntry();
        // tab
        // .add("Shoulder P", Constants.ShoulderConstants.kP)
        // .getEntry();
        // tab
        // .add("Shoulder I", Constants.ShoulderConstants.kI)
        // .getEntry();
        // tab
        // .add("Shoulder D", Constants.ShoulderConstants.kD)
        // .getEntry();
        // tab
        // .add("Shoulder FF", Constants.ShoulderConstants.kFF)
        // .getEntry();

        // tab
        // .add("Wrist kS", Constants.ShoulderConstants.kSVolts)
        // .getEntry();
        // tab
        // .add("Wrist kG", Constants.ShoulderConstants.kGVolts)
        // .getEntry();
        // tab
        // .add("Wrist kV", Constants.ShoulderConstants.kVVoltSecondPerRad)
        // .getEntry();
        // tab
        // .add("Wrist kA", Constants.ShoulderConstants.kAVoltSecondSquaredPerRad)
        // .getEntry();
        // tab
        // .add("Wrist P", Constants.ShoulderConstants.kP)
        // .getEntry();
        // tab
        // .add("Wrist I", Constants.ShoulderConstants.kI)
        // .getEntry();
        // tab
        // .add("Wrist D", Constants.ShoulderConstants.kD)
        // .getEntry();
        // tab
        // .add("Wrist FF", Constants.ShoulderConstants.kFF)
        // .getEntry();
    }

    @Override
    public void update() {
        mOperatorHasCube.setBoolean(claw.hasCube());
        mOperatorHasCone.setBoolean(claw.hasCone());

        mShoulderSpeed.setDouble(shoulder.getSpeed());
        mShoulderAngle.setDouble(shoulder.getPosition());
        mShoulderIsMoving.setBoolean(shoulder.isMoving());
        mShoulderCurrent.setDouble(shoulder.getOutputCurrent());
        mShoulderTemp.setDouble(shoulder.getMotorTemperature());
        mShoulderArbitraryFF.setDouble(shoulder.getArbitraryFF());

        mClawSpeed.setDouble(claw.getClawSpeed());
        mClawIsIntaking.setBoolean(claw.isIntaking());
        mClawCurrent.setDouble(claw.getCurrent());

        mWristSpeed.setDouble(wrist.getSpeed());
        mWristAngle.setDouble(wrist.getPosition());
        mWristIsMoving.setBoolean(wrist.isMoving());
        mWristCurrent.setDouble(wrist.getOutputCurrent());
        mWristTemp.setDouble(wrist.getMotorTemperature());
        mWristArbitraryFF.setDouble(wrist.getArbitraryFF());

        arm.setShoulderPosition(mSetShoulderPos.getDouble(0.0));
        arm.setWristPosition(mSetWristPos.getDouble(0.0));
        claw.setSpeed(mSetClawSpeed.getDouble(0.0));

    }
}
