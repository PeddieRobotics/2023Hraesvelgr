package frc.robot.Shuffleboard.tabs;

import java.util.Enumeration;
import java.util.Hashtable;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shuffleboard.FieldView;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class OperatorTab extends ShuffleboardTabBase {
    private Arm arm = Arm.getInstance();
    private Claw claw = Claw.getInstance();
    private Wrist wrist = Wrist.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Autonomous autonomous = Autonomous.getInstance();

    private FieldView fieldView;
    private GenericEntry mArmState, mStowingIntake;
    private ComplexWidget mAutoChooser;

    // Sendable Chooser
    private SendableChooser<Command> autoRoutineSelector;

    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        fieldView = new FieldView();
        tab.add(fieldView.getField()).
        withSize(8, 5).
        withPosition(0,0);

        autoRoutineSelector = new SendableChooser<Command>();

        try {
            mArmState = tab.add("Arm State", "Home").getEntry();
            mStowingIntake = tab.add("Stowing Intake", false).getEntry();
        } catch(IllegalArgumentException e){
        }

    }

    @Override
    public void update() {
        fieldView.update();
        mArmState.setString(arm.getState().toString());
        arm.setAllowStowIntake(mStowingIntake.getBoolean(false));
    }

    public void setupAutoSelector(){
        Hashtable<String,Command> autoRoutines = autonomous.getAutoRoutines();
        Enumeration<String> e = autoRoutines.keys();

        while (e.hasMoreElements()) {
            String autoRoutineName = e.nextElement();
            autoRoutineSelector.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        }
        mAutoChooser = tab.add("Auto routine", autoRoutineSelector).withSize(3,1).withPosition(9,3);

    }

    public Command getAutonomousCommand(){
        return autoRoutineSelector.getSelected();
    }

}
