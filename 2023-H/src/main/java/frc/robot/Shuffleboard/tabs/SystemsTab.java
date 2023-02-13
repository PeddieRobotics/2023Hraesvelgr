package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

public class SystemsTab extends ShuffleboardTabBase {
    private Claw claw = Claw.getInstance();
    private LimelightFront LLFront = LimelightFront.getInstance();
    private LimelightBack LLBack = LimelightBack.getInstance();
    private GenericEntry mTx;
    private GenericEntry mTy;
    private GenericEntry mTa;
    private GenericEntry mHasCube;
    private GenericEntry mHasCone;
    private GenericEntry mLimelightHasTarget;
    private GenericEntry mPrimaryTagID;

    public void createEntries() {
        tab = Shuffleboard.getTab("System");

        mHasCube = tab
                .add("hasCube", false)
                .withSize(3, 2)
                .withPosition(2, 1)
                .getEntry();
        mHasCone = tab
                .add("hasCone", false)
                .withSize(3, 2)
                .withPosition(2, 2)
                .getEntry();
        mLimelightHasTarget = tab
                .add("hasCube", false)
                .withSize(3, 2)
                .withPosition(2, 1)
                .getEntry();
        mTx = tab
                .add("tx", 0.0)
                .withSize(3, 2)
                .withPosition(2, 1)
                .getEntry();
        mTy = tab
                .add("ty", 0.0)
                .withSize(3, 2)
                .withPosition(2, 1)
                .getEntry();
        mTa = tab
                .add("ta", 0.0)
                .withSize(3, 2)
                .withPosition(2, 1)
                .getEntry();
        mPrimaryTagID = tab
                .add("primaryTagID", 1)
                .withSize(3, 2)
                .withPosition(3, 2)
                .getEntry();
    }

    @Override
    public void update() {
        mHasCube.setBoolean(claw.hasCube());
        mHasCone.setBoolean(claw.hasCone());
        mTx.setDouble(LLFront.getTx());
        mTy.setDouble(LLFront.getTy());
        mTa.setDouble(LLFront.getTa());
        mLimelightHasTarget.setBoolean(LLFront.hasTarget());
        mPrimaryTagID.setInteger(LLBack.getTargetAprilTagID());
    }
}
