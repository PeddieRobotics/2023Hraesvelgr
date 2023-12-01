package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.LimelightFront;

public class LimelightFrontTab extends ShuffleboardTabBase {
    private LimelightFront LLFront = LimelightFront.getInstance();

    private GenericEntry mTx;
    private GenericEntry mTy;
    private GenericEntry mTa;
    private GenericEntry mDistToTarget;
    private GenericEntry mHasTarget;
    private GenericEntry mTagsSeen;
    private GenericEntry mBotposeX;
    private GenericEntry mBotposeY;
    private GenericEntry mBotposeTheta;

    public LimelightFrontTab(){
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Limelight Front");

        try{
                // tab.addCamera("Stream", "LL Front", "mjpg:http://10.58.95.11:5800")
                // .withSize(9, 6)
                // .withPosition(0, 1);

                mTx = tab.add("tx", 0.0)
                        .getEntry();
                mTy = tab.add("ty", 0.0)
                        .getEntry();
                mTa = tab.add("ta", 0.0)
                        .getEntry();
                mDistToTarget = tab.add("distToTarget", 0.0)
                        .getEntry();
                mHasTarget = tab.add("hasTarget", false)
                        .getEntry();
                mTagsSeen = tab.add("tagsSeen", 0)
                        .getEntry();
                mBotposeX = tab.add("Botpose X", 0.0)
                        .getEntry();
                mBotposeY = tab.add("Botpose Y", 0.0)
                        .getEntry();
                mBotposeTheta = tab.add("Botpose Theta", 0.0)
                        .getEntry();

        } catch(IllegalArgumentException e){
        }
    }

    @Override
    public void update() {
        try{
                mTx.setDouble(LLFront.getTx());
                mTy.setDouble(LLFront.getTy());
                mTa.setDouble(LLFront.getTa());
                mHasTarget.setBoolean(LLFront.hasTarget());
                mDistToTarget.setDouble(LLFront.getDistance());
                mTagsSeen.setInteger(LLFront.getTagsSeen());
                mBotposeX.setDouble(LLFront.getBotpose().getX());
                mBotposeY.setDouble(LLFront.getBotpose().getY());
                mBotposeTheta.setDouble(LLFront.getBotpose().getRotation().getDegrees());
        } catch(NullPointerException e){
        }
    }

}
