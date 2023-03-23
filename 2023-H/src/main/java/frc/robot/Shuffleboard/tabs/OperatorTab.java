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
import frc.robot.utils.OperatorOI;

public class OperatorTab extends ShuffleboardTabBase {
    private Arm arm = Arm.getInstance();
    private Claw claw = Claw.getInstance();
    private Wrist wrist = Wrist.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Autonomous autonomous = Autonomous.getInstance();
    private OperatorOI operatorOI = OperatorOI.getInstance();

    private FieldView fieldView;
    private GenericEntry mArmState, mUsePreScorePose, mClawState, mGamepieceState;
    private ComplexWidget mAutoChooser;

    // Sendable Chooser
    private SendableChooser<Command> autoRoutineSelector;

    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        // fieldView = new FieldView();
        // tab.add(fieldView.getField()).
        // withSize(8, 5).
        // withPosition(0,0);

        autoRoutineSelector = new SendableChooser<Command>();

        try {
            mArmState = tab.add("Arm State", "Home").withSize(2,2).withPosition(24,1).getEntry();
            mClawState = tab
            .add("Game piece?", "Empty").withSize(2,2).withPosition(22,1)
            .getEntry();
            mGamepieceState = tab.add("Has gamepiece", false).withSize(6,6).withPosition(0,0).getEntry();
            tab.addCamera("Stream", "LL Front", "mjpg:http://10.58.95.11:5800")
                .withSize(9, 6)
                .withPosition(6, 0);
        } catch(IllegalArgumentException e){
        }

    }

    @Override
    public void update() {
        // fieldView.update();
        mArmState.setString(arm.getState().toString());
        mClawState.setString(claw.getState().toString());
        mGamepieceState.setBoolean(claw.hasGamepiece());
    }

    public void setupAutoSelector(){
        Hashtable<String,Command> autoRoutines = autonomous.getAutoRoutines();
        Enumeration<String> e = autoRoutines.keys();

        while (e.hasMoreElements()) {
            String autoRoutineName = e.nextElement();
            autoRoutineSelector.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        }
        mAutoChooser = tab.add("Auto routine", autoRoutineSelector).withSize(5,2).withPosition(16,1); // comp settings: withPosition(16,1);

    }

    public Command getAutonomousCommand(){
        return autoRoutineSelector.getSelected();
    }

}
