package frc.robot.Shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Shuffleboard.tabs.*;

public class ShuffleboardMain {
    private static ShuffleboardMain shuffleboard;

    private ArrayList<ShuffleboardTabBase> tabs = new ArrayList<ShuffleboardTabBase>();

    private OperatorTab operatorTab;
    private AngleOverridesTab angleOverridesTab;

    private ClawTab clawTab;
    private ShoulderTab shoulderTab;
    private WristTab wristTab;
    private DrivetrainTab drivetrainTab;
    private LimelightFrontTab limelightFrontTab;
    //private LimelightBackTab limelightBackTab;
    private SystemsTab systemsTab;

    public ShuffleboardMain() {

    }

    public static ShuffleboardMain getInstance() {
        if (shuffleboard == null) {
            shuffleboard = new ShuffleboardMain();
        }
        return shuffleboard;
    }

    public void setupTeleop(){
        operatorTab = new OperatorTab();

        tabs.add(operatorTab);

        angleOverridesTab = new AngleOverridesTab();
        tabs.add(angleOverridesTab);

        systemsTab = new SystemsTab();
        tabs.add(systemsTab);

        limelightFrontTab = new LimelightFrontTab();

        tabs.add(limelightFrontTab);
        
        //limelightBackTab = new LimelightBackTab();
        //tabs.add(limelightBackTab);

        for (ShuffleboardTabBase tab : tabs) {
            tab.createEntries();
        }

    }

    public void setupTestMode(){
        operatorTab = new OperatorTab();
        tabs.add(operatorTab);

        angleOverridesTab = new AngleOverridesTab();
        tabs.add(angleOverridesTab);

        clawTab = new ClawTab();
        tabs.add(clawTab);

        shoulderTab = new ShoulderTab();
        tabs.add(shoulderTab);
        
        wristTab = new WristTab();
        tabs.add(wristTab);
        
        drivetrainTab = new DrivetrainTab();
        tabs.add(drivetrainTab);

        limelightFrontTab = new LimelightFrontTab();
        tabs.add(limelightFrontTab);
        
        //limelightBackTab = new LimelightBackTab();
        //tabs.add(limelightBackTab);

        for (ShuffleboardTabBase tab : tabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : tabs) {
            tab.update();
        }
        
    }

}