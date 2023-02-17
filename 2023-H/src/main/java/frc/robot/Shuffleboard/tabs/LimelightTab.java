package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

public class LimelightTab extends ShuffleboardTabBase {
    private LimelightFront LLFront = LimelightFront.getInstance();
    private LimelightBack LLBack = LimelightBack.getInstance();

    private GenericEntry mTxFront;
    private GenericEntry mTyFront;
    private GenericEntry mTaFront;
    private GenericEntry mDistToTargetFront;
    private GenericEntry mLimelightHasTargetFront;
    private GenericEntry mTagsSeenFront;
    private GenericEntry mBotposeFrontX;
    private GenericEntry mBotposeFrontY;
    private GenericEntry mBotposeFrontTheta;

    private GenericEntry mTxBack;
    private GenericEntry mTyBack;
    private GenericEntry mTaBack;
    private GenericEntry mDistToTargetBack;
    private GenericEntry mLimelightHasTargetBack;
    private GenericEntry mTagsSeenBack;
    private GenericEntry mBotposeBackX;
    private GenericEntry mBotposeBackY;
    private GenericEntry mBotposeBackTheta;

    public void createEntries() {
        tab = Shuffleboard.getTab("Limelight");

        mTxFront = tab
                .add("tx Front", 0.0)
                .getEntry();
        mTyFront = tab
                .add("ty Front", 0.0)
                .getEntry();
        mTaFront = tab
                .add("ta Front", 0.0)
                .getEntry();
        mDistToTargetFront = tab
                .add("distToTarget Front", 0.0)
                .getEntry();
        mLimelightHasTargetFront = tab
                .add("hasTarget Front", false)
                .getEntry();
        mTagsSeenFront = tab
                .add("tagsSeen Front", 0)
                .getEntry();
        mBotposeFrontX = tab
                .add("Botpose X Front", 0.0)
                .getEntry();
        mBotposeFrontY = tab
                .add("Botpose Y Front", 0.0)
                .getEntry();
        mBotposeFrontTheta = tab
                .add("Botpose Theta Front", 0.0)
                .getEntry();

        mTxBack = tab
                .add("tx Back", 0.0)
                .getEntry();
        mTyBack = tab
                .add("ty Back", 0.0)
                .getEntry();
        mTaBack = tab
                .add("ta Back", 0.0)
                .getEntry();
        mDistToTargetBack = tab
                .add("distToTarget Back", 0.0)
                .getEntry();
        mLimelightHasTargetBack = tab
                .add("hasTarget Back", false)
                .getEntry();
        mTagsSeenBack = tab
                .add("tagsSeen Back", 0)
                .getEntry();
        mBotposeBackX = tab
                .add("Botpose X Back", 0.0)
                .getEntry();
        mBotposeBackY = tab
                .add("Botpose Y Back", 0.0)
                .getEntry();
        mBotposeBackTheta = tab
                .add("Botpose Theta Back", 0.0)
                .getEntry();
    }

    @Override
    public void update() {

        mTxFront.setDouble(LLFront.getTx());
        mTyFront.setDouble(LLFront.getTy());
        mTaFront.setDouble(LLFront.getTa());
        mLimelightHasTargetFront.setBoolean(LLFront.hasTarget());
        mDistToTargetFront.setDouble(LLFront.getDistance());
        mTagsSeenFront.setInteger(LLFront.getTagsSeen());
        mBotposeFrontX.setDouble(LLFront.getBotpose().getX());
        mBotposeFrontY.setDouble(LLFront.getBotpose().getY());
        mBotposeFrontTheta.setDouble(LLFront.getBotpose().getRotation().getDegrees());

        mTxBack.setDouble(LLBack.getTx());
        mTyBack.setDouble(LLBack.getTy());
        mTaBack.setDouble(LLBack.getTa());
        mLimelightHasTargetBack.setBoolean(LLBack.hasTarget());
        mDistToTargetBack.setDouble(LLBack.getDistance());
        mTagsSeenBack.setInteger(LLBack.getTagsSeen());
        mBotposeBackX.setDouble(LLBack.getBotpose().getX());
        mBotposeBackY.setDouble(LLBack.getBotpose().getY());
        mBotposeBackTheta.setDouble(LLBack.getBotpose().getRotation().getDegrees());
    }
}
