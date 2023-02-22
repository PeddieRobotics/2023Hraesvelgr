package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

public class SystemsTab extends ShuffleboardTabBase {
    private Claw claw = Claw.getInstance();
    private LimelightFront LLFront = LimelightFront.getInstance();
    //private LimelightBack LLBack = LimelightBack.getInstance();

    public void createEntries() {
        tab = Shuffleboard.getTab("Systems");
    }

    @Override
    public void update() {
    }

}
