package frc.robot.Shuffleboard.tabs;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.LimelightBack;

public class SystemsTab extends ShuffleboardTabBase{
    private LimelightBack LL;
    private NetworkTableEntry testTab;
    private NetworkTableEntry mOperatorHasCube;
    public void createEntries() {
        tab = Shuffleboard.getTab("System");
        testTab = tab
                .add("test", false)
                .withSize(3, 2)
                .withPosition(2, 1)
                .getEntry();
        // mOperatorHasCube = tab
        //         .add("hasCube", false)
        //         .withSize(3, 2)
        //         .withPosition(2, 1)
        //         .getEntry();
    }

    @Override
    public void update() {
        testTab.setBoolean(true);
        //mOperatorHasCube.setBoolean(LL.hasCube());
    }
}
