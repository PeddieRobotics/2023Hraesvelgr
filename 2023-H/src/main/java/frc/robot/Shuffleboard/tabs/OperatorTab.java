package frc.robot.Shuffleboard.tabs;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LimelightFront;

public class OperatorTab extends ShuffleboardTabBase{
    private LimelightFront LL = LimelightFront.getInstance();
    private Arm arm = Arm.getInstance();
    private GenericEntry mOperatorHasCube;
    private GenericEntry mOperatorHasCone;
    private GenericEntry mWristSpeed;
    private GenericEntry mShoulderSpeed;
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
        mWristSpeed = tab
                .add("wristSpeed", 0.0)
                .withSize(3, 2)
                .withPosition(2, 3)
                .getEntry();
        mShoulderSpeed = tab
                .add("shoulderSpeed", 0.0)
                .withSize(3, 2)
                .withPosition(2, 4)
                .getEntry();
    }

    @Override
    public void update() {
        mOperatorHasCube.setBoolean(LL.targetIsCube());
        mOperatorHasCone.setBoolean(LL.targetIsCone());
        mWristSpeed.setDouble(arm.getWristSpeed());
        mShoulderSpeed.setDouble(arm.getShoulderSpeed());
    }
}
