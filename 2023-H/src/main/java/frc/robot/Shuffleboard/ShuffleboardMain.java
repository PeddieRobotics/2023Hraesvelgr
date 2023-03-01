package frc.robot.Shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
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
    private LimelightBackTab limelightBackTab;
    private SystemsTab systemsTab;

    public ShuffleboardMain() {

    }

    public static ShuffleboardMain getInstance() {
        if (shuffleboard == null) {
            shuffleboard = new ShuffleboardMain();
        }
        return shuffleboard;
    }

    public void setupCompetitionMode(){
        operatorTab = new OperatorTab();

        tabs.add(operatorTab);

        angleOverridesTab = new AngleOverridesTab();
        tabs.add(angleOverridesTab);

        systemsTab = new SystemsTab();
        tabs.add(systemsTab);

        limelightFrontTab = new LimelightFrontTab();

        tabs.add(limelightFrontTab);
        
        limelightBackTab = new LimelightBackTab();
        tabs.add(limelightBackTab);

        for (ShuffleboardTabBase tab : tabs) {
            tab.createEntries();
        }

    }

    public void setupDebugMode(){
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
        
        limelightBackTab = new LimelightBackTab();
        tabs.add(limelightBackTab);

        for (ShuffleboardTabBase tab : tabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : tabs) {
            tab.update();
        }
        
    }

    public Command getAutonomousCommand(){
        return operatorTab.getAutonomousCommand();
    }

    public void setupAutoSelector(){
        operatorTab.setupAutoSelector();
    }

}