package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Claw;

public class ClawTab extends ShuffleboardTabBase {
        private Claw claw = Claw.getInstance();

        private GenericEntry mSpeed, mCurrent, mTemp, mVoltage, mOpenLoopToggle, mSpeedSetpoint, mState,
        mFrontSensor, mBackSensor, mUseSensors;

        public ClawTab() {
        }

        public void createEntries() {
                tab = Shuffleboard.getTab("Claw");

                try {
                        mSpeed = tab.add("Speed", 0.0)
                                        .getEntry();
                        mCurrent = tab.add("Current", 0.0)
                                        .getEntry();
                        mTemp = tab.add("Temperature", 0.0)
                                        .getEntry();
                        mVoltage = tab.add("Voltage", 0.0)
                                        .getEntry();
                        mOpenLoopToggle = tab.add("Open Loop Toggle", false)
                                        .withWidget(BuiltInWidgets.kToggleButton)
                                        .getEntry();
                        mSpeedSetpoint = tab.add("Speed Setpoint", 0.0)
                                        .getEntry();
                        mState = tab
                                        .add("Game piece?", "Empty")
                                        .getEntry();
                        mFrontSensor = tab
                                .add("Front sensor", false)
                                .getEntry();
                        mBackSensor = tab
                                .add("Back sensor", false)
                                .getEntry();
                        mUseSensors = tab.add("Use sensors", true)
                                .withWidget(BuiltInWidgets.kToggleButton)
                                .getEntry();
                } catch (IllegalArgumentException e) {
                }

        }

        @Override
        public void update() {
                try {
                        mSpeed.setDouble(claw.getSpeed());
                        mCurrent.setDouble(claw.getOutputCurrent());
                        mTemp.setDouble(claw.getMotorTemperature());
                        mVoltage.setDouble(claw.getVoltage());
                        mState.setString(claw.getState().toString());
                        mFrontSensor.setBoolean(claw.isFrontSensor());
                        mBackSensor.setBoolean(claw.isBackSensor());
                        
                        claw.setUseSensors(mUseSensors.getBoolean(true));

                        if (mOpenLoopToggle.getBoolean(false)) {
                                claw.setSpeed(mSpeedSetpoint.getDouble(0.0));
                        }
                } catch (NullPointerException e) {
                }

        }

}
