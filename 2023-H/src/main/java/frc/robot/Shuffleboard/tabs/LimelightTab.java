package frc.robot.Shuffleboard.tabs;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

public class LimelightTab extends ShuffleboardTabBase{
    private LimelightFront LLFront = LimelightFront.getInstance();
    private LimelightBack LLBack = LimelightBack.getInstance();
    private GenericEntry mOperatorHasCube;
    private GenericEntry mOperatorHasCone;
    private GenericEntry mLimelightHasTarget;
    private GenericEntry mPrimaryTagID;
    private GenericEntry mDistToTarget;
    private GenericEntry mPipelineFront;
    private GenericEntry mPipelineBack;
    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        mOperatorHasCube = tab
                .add("hasCube", false)
                .withSize(3, 2)
                .withPosition(2, 1)
                .getEntry();
        mOperatorHasCone = tab
                .add("hasCone", false)
                .withSize(3, 2)
                .withPosition(2, 2)
                .getEntry();
        mLimelightHasTarget = tab
                .add("hasCube", false)
                .withSize(3, 2)
                .withPosition(2, 1)
                .getEntry();
        mPrimaryTagID = tab
                .add("primaryTagID", 1)
                .withSize(3, 2)
                .withPosition(3, 2)
                .getEntry();
        mDistToTarget = tab
                .add("distanceToTarget", 0.0)
                .withSize(3, 2)
                .withPosition(3, 2)
                .getEntry();
        mPipelineFront = tab
                .add("pipeline", 0)
                .withSize(3, 2)
                .withPosition(3, 2)
                .getEntry();
        mPipelineBack = tab
                .add("pipeline", 0)
                .withSize(3, 2)
                .withPosition(3, 2)
                .getEntry();
    }

    @Override
    public void update() {
        mOperatorHasCube.setBoolean(LLFront.targetIsCube());
        mOperatorHasCone.setBoolean(LLFront.targetIsCone());
        mLimelightHasTarget.setBoolean(LLFront.hasTarget());
        mPrimaryTagID.setInteger(LLBack.getTargetAprilTagID());
        mPipelineFront.setDouble(LLFront.getPipeline());
        mPipelineBack.setDouble(LLBack.getPipeline());
        mDistToTarget.setDouble(LLFront.getDistance());
    }
}
