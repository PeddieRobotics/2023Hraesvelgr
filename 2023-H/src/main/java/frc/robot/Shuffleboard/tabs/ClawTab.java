package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Claw;
import frc.robot.utils.OI;
import frc.robot.utils.Constants.WristConstants;

public class ClawTab extends ShuffleboardTabBase {
    private Claw claw = Claw.getInstance();

    private GenericEntry mSpeed, mCurrent, mTemp, mOpenLoopToggle, mSpeedSetpoint;

    public ClawTab(){
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Claw");

        try{
        mSpeed = tab.add("Speed", 0.0)
                .getEntry();
        mCurrent = tab.add("Current", 0.0)
                .getEntry();
        mTemp = tab.add("Temperature", 0.0)
                .getEntry();
        mOpenLoopToggle = tab.add("Open Loop Toggle", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
        mSpeedSetpoint = tab.add("Speed Setpoint", 0.0)
                .getEntry();
        } catch(IllegalArgumentException e){
        }

    }

    @Override
    public void update() {
        try{
                mSpeed.setDouble(claw.getSpeed());
                mCurrent.setDouble(claw.getOutputCurrent());
                mTemp.setDouble(claw.getMotorTemperature());

                if(mOpenLoopToggle.getBoolean(false)){
                        claw.setSpeed(mSpeedSetpoint.getDouble(0.0));
                }
        } catch(NullPointerException e){
        }

    }


}