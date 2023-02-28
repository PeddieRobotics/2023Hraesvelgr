package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.LimelightBack;

public class LimelightBackTab extends ShuffleboardTabBase {
    private LimelightBack LLBack = LimelightBack.getInstance();

    private GenericEntry mTx;
    private GenericEntry mTy;
    private GenericEntry mTa;
    private GenericEntry mDistToTarget;
    private GenericEntry mLimelightHasTarget;
    private GenericEntry mTagsSeen;
    private GenericEntry mBotposeX;
    private GenericEntry mBotposeY;
    private GenericEntry mBotposeTheta;

public LimelightBackTab(){
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Limelight Back");

        tab.addCamera("Stream", "LL Back", "mjpg:http://10.58.95.12:5800")
        .withSize(9, 6)
        .withPosition(0, 1);
        
        mTx = tab.add("tx", 0.0)
                .getEntry();
        mTy = tab.add("ty", 0.0)
                .getEntry();
        mTa = tab.add("ta", 0.0)
                .getEntry();
        mDistToTarget = tab.add("distToTarget", 0.0)
                .getEntry();
        mLimelightHasTarget = tab.add("hasTarget", false)
                .getEntry();
        mTagsSeen = tab.add("tagsSeen", 0)
                .getEntry();
        mBotposeX = tab
                .add("Botpose X", 0.0)
                .getEntry();
        mBotposeY = tab
                .add("Botpose Y", 0.0)
                .getEntry();
        mBotposeTheta = tab
                .add("Botpose Theta", 0.0)
                .getEntry();

    }

    @Override
    public void update() {
        try{
            mTx.setDouble(LLBack.getTx());
            mTy.setDouble(LLBack.getTy());
            mTa.setDouble(LLBack.getTa());
            mLimelightHasTarget.setBoolean(LLBack.hasTarget());
            mDistToTarget.setDouble(LLBack.getDistance());
            mTagsSeen.setInteger(LLBack.getTagsSeen());
            mBotposeX.setDouble(LLBack.getBotpose().getX());
            mBotposeY.setDouble(LLBack.getBotpose().getY());
            mBotposeTheta.setDouble(LLBack.getBotpose().getRotation().getDegrees());
        } catch(NullPointerException e){
        }
    }

}
