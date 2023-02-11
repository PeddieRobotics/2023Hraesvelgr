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
    private GenericEntry mOperatorHasCube;
    private GenericEntry mOperatorHasCone;
    private GenericEntry mLimelightHasTarget;
    private GenericEntry mPrimaryTagID;

    public void createEntries() {
        tab = Shuffleboard.getTab("System");

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
    }

    @Override
    public void update() {
        mOperatorHasCube.setBoolean(claw.hasCube());
        mOperatorHasCone.setBoolean(claw.hasCone());
        mLimelightHasTarget.setBoolean(LLFront.hasTarget());
        mPrimaryTagID.setInteger(LLBack.getTargetAprilTagID());
    }
}
