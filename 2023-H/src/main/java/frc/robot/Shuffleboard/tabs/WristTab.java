package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.OI;
import frc.robot.utils.Constants.WristConstants;

public class WristTab extends ShuffleboardTabBase {
    private Wrist wrist = Wrist.getInstance();

    private GenericEntry mSpeed, mAngle, mCurrent, mTemp, mArbitraryFF, mOpenLoopToggle, mPIDToggle,
    mkG, mkV, mkA, mkP, mkI, mkD, mkIz, mPIDSetpoint;

    public WristTab(){
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Wrist");

        try{
        mSpeed = tab.add("Speed", 0.0)
                .getEntry();
        mAngle = tab.add("Angle", 0.0)
                .getEntry();
        mCurrent = tab.add("Current", 0.0)
                .getEntry();
        mTemp = tab.add("Temperature", 0.0)
                .getEntry();

        mOpenLoopToggle = tab.add("Open Loop Toggle", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();
        mPIDToggle = tab.add("PID Toggle", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();
        mPIDSetpoint = tab.add("PID Setpoint", WristConstants.kIz)
            .getEntry();
        mArbitraryFF = tab.add("Arb FF", 0.0)
            .getEntry();
                
        mkG = tab.add("kG", WristConstants.kGVolts)
            .getEntry();
        mkV = tab.add("kV", WristConstants.kVVoltSecondPerRad)
            .getEntry();
        mkA = tab.add("kA", WristConstants.kAVoltSecondSquaredPerRad)
            .getEntry();
        mkP = tab.add("kP", WristConstants.kP)
            .getEntry();
        mkI = tab.add("kI", WristConstants.kI)
            .getEntry();
        mkIz = tab.add("IZone", WristConstants.kIz)
            .getEntry();
        mkD = tab.add("kD", WristConstants.kD)
            .getEntry();
        } catch(IllegalArgumentException e){
        }

    }

    @Override
    public void update() {
        try{
            mSpeed.setDouble(wrist.getSpeed());
            mAngle.setDouble(wrist.getPosition());
            mCurrent.setDouble(wrist.getOutputCurrent());
            mTemp.setDouble(wrist.getMotorTemperature());
            mArbitraryFF.setDouble(wrist.getArbitraryFF());

            if (mOpenLoopToggle.getBoolean(false)) {
                wrist.setPercentOutput(OI.getInstance().getArmSpeed());
            }
            else if (mPIDToggle.getBoolean(false)) {
                wrist.setPIDController(mkP.getDouble(WristConstants.kP),
                        mkI.getDouble(WristConstants.kI),
                        mkD.getDouble(WristConstants.kD),
                        mkIz.getDouble(WristConstants.kIz));

                wrist.setWristFeedforward(
                        mkG.getDouble(0.0),
                        mkV.getDouble(0.0),
                        mkA.getDouble(0.0));

                wrist.setPosition(mPIDSetpoint.getDouble(0.0));

            }
        } catch(NullPointerException e){
        }
    }


}
