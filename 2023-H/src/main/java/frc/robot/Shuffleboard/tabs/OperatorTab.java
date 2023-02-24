package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.FieldView;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class OperatorTab extends ShuffleboardTabBase {
    private Arm arm = Arm.getInstance();
    private Claw claw = Claw.getInstance();
    private Wrist wrist = Wrist.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private FieldView fieldView;
    private GenericEntry mArmState;

    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        fieldView = new FieldView();
        tab.add(fieldView.getField()).
        withSize(8, 5).
        withPosition(0,0);

        try {
            mArmState = tab
                            .add("Arm State", "Home")
                            .getEntry();
        } catch(IllegalArgumentException e){
        }

    }

    @Override
    public void update() {
        fieldView.update();
        mArmState.setString(arm.getState().toString());
    }

}
