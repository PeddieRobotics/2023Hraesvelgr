package frc.robot.Shuffleboard.tabs;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.LimelightBack;

public class SystemsTab extends ShuffleboardTabBase{
    private LimelightBack LL;
    private NetworkTable testTab;
    private NetworkTableEntry mOperatorHasCube;
    public void createEntries() {
        tab = Shuffleboard.getTab("System");
        testTab = tab
                .getDoubleTopic("test").subscribe(0.0);
        // mOperatorHasCube = tab
        //         .add("hasCube", false)
        //         .withSize(3, 2)
        //         .withPosition(2, 1)
        //         .getEntry();
    }

    @Override
    public void update() {
        testTab.setDouble(0.0);
        //mOperatorHasCube.setBoolean(LL.hasCube());
    }
}
