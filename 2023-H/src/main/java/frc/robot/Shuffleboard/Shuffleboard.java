package frc.robot.Shuffleboard;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Shuffleboard.tabs.*;

public class Shuffleboard {
    private static Shuffleboard shuffleboard;
    public final boolean isTestMode = true;

    public static Shuffleboard getInstance() {
        if (shuffleboard == null) {
            shuffleboard = new Shuffleboard();
        }
        return shuffleboard;
    }

    private ArrayList<ShuffleboardTabBase> tabs = new ArrayList<ShuffleboardTabBase>();

    private OperatorTab operatorTab;

    public Shuffleboard() {
        operatorTab = new OperatorTab();
        tabs.add(operatorTab);
        if (isTestMode) {
            List<ShuffleboardTabBase> optionalTabs = List.of(
                    new ArmTab(),
                    new LimelightTab());
            tabs.addAll(optionalTabs);

        } else {
            tabs.add(new SystemsTab());
        }

        for (ShuffleboardTabBase tab : tabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : tabs) {
            tab.update();
        }
    }

    public ShuffleboardTab getOperatorTab() {
        return operatorTab.getTab();
    }
}