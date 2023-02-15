package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

public class LimelightTab extends ShuffleboardTabBase {
    private LimelightFront LLFront = LimelightFront.getInstance();
    private LimelightBack LLBack = LimelightBack.getInstance();
    private GenericEntry mTx;
    private GenericEntry mTy;
    private GenericEntry mTa;
    private GenericEntry mLimelightHasTarget;
    private GenericEntry mPrimaryTagID;
    private GenericEntry mDistToTarget;
    private GenericEntry mPipelineFront;
    private GenericEntry mPipelineBack;

    public void createEntries() {
        tab = Shuffleboard.getTab("Limelight");

        mLimelightHasTarget = tab
                .add("hasTarget", false)
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
        // mPrimaryTagID = tab TODO
        // .add("primaryTagID", 1)
        // .withSize(3, 2)
        // .withPosition(3, 2)
        // .getEntry();
        mDistToTarget = tab
                .add("distanceToTarget", 0.0)
                .withSize(3, 2)
                .withPosition(3, 2)
                .getEntry();
        mPipelineFront = tab
                .add("pipelineFront", 0)
                .withSize(3, 2)
                .withPosition(3, 2)
                .getEntry();
        mPipelineBack = tab
                .add("pipelineBack", 0)
                .withSize(3, 2)
                .withPosition(3, 2)
                .getEntry();
    }

    @Override
    public void update() {
        mLimelightHasTarget.setBoolean(LLFront.hasTarget());
        mTx.setDouble(LLFront.getTx());
        mTy.setDouble(LLFront.getTy());
        mTa.setDouble(LLFront.getTa());
        mDistToTarget.setDouble(LLFront.getDistance());
        // mPrimaryTagID.setInteger(LLBack.getTargetAprilTagID());
    }
}
