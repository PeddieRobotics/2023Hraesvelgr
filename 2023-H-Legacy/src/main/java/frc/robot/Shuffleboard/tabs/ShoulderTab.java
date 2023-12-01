package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.ShoulderConstants;

public class ShoulderTab extends ShuffleboardTabBase {
        private Shoulder shoulder = Shoulder.getInstance();

        private GenericEntry mSpeed, mAngle, mCurrent, mTemp, mVoltage, mArbitraryFF, mOpenLoopToggle, mPIDToggle,
                        mkG, mkV, mkA, mkP, mkI, mkD, mkIz, mkFF, mPIDSetpoint, mSmartMotionAngleTol, mSmartMotionMinVel,
                        mSmartMotionMaxVel, mSmartMotionMaxAccel;

        public ShoulderTab() {
        }

        public void createEntries() {
                tab = Shuffleboard.getTab("Shoulder");

                try {
                        mSpeed = tab
                                        .add("Speed", 0.0)
                                        .getEntry();
                        mAngle = tab
                                        .add("Angle", 0.0)
                                        .getEntry();
                        mCurrent = tab
                                        .add("Current", 0.0)
                                        .getEntry();
                        mTemp = tab
                                        .add("Temperature", 0.0)
                                        .getEntry();
                        mVoltage = tab
                                        .add("Voltage", 0.0)
                                        .getEntry();
                        mOpenLoopToggle = tab.add("Open Loop Toggle", false)
                                        .withWidget(BuiltInWidgets.kToggleButton)
                                        .getEntry();
                        mPIDToggle = tab.add("PID Toggle", false)
                                        .withWidget(BuiltInWidgets.kToggleButton)
                                        .getEntry();
                        mPIDSetpoint = tab.add("PID Setpoint", 0.0)
                                        .getEntry();
                        mArbitraryFF = tab.add("Arb FF", 0.0)
                                        .getEntry();

                        mkG = tab.add("kG", ShoulderConstants.kGVolts)
                                        .getEntry();
                        mkV = tab.add("kV", ShoulderConstants.kVVoltSecondPerRad)
                                        .getEntry();
                        mkA = tab.add("kA", ShoulderConstants.kAVoltSecondSquaredPerRad)
                                        .getEntry();
                        mkP = tab.add("kP", ShoulderConstants.kP)
                                        .getEntry();
                        mkI = tab.add("kI", ShoulderConstants.kI)
                                        .getEntry();
                        mkIz = tab.add("IZone", ShoulderConstants.kIz)
                                        .getEntry();
                        mkD = tab.add("kD", ShoulderConstants.kD)
                                        .getEntry();
                        mkFF = tab.add("kFF", ShoulderConstants.kFF)
                                        .getEntry();

                        mSmartMotionAngleTol = tab.add("S.M. Setpoint Tol", ShoulderConstants.kSmartMotionRegularSetpointTol)
                                        .getEntry();
                        mSmartMotionMinVel = tab.add("S.M. Min Vel", ShoulderConstants.kSmartMotionRegularMinVel)
                                        .getEntry();
                        mSmartMotionMaxVel = tab.add("S.M. Max Vel", ShoulderConstants.kSmartMotionRegularMaxVel)
                                        .getEntry();
                        mSmartMotionMaxAccel = tab.add("S.M. Max Accel", ShoulderConstants.kSmartMotionRegularMaxAccel)
                                        .getEntry();
                        // mLimitSensor = tab.add("Limit sensor", false)
                        //                 .getEntry();
                } catch (IllegalArgumentException e) {
                }

        }

        @Override
        public void update() {
                try {
                        mSpeed.setDouble(shoulder.getVelocity());
                        mAngle.setDouble(shoulder.getPosition());
                        mCurrent.setDouble(shoulder.getOutputCurrent());
                        mTemp.setDouble(shoulder.getMotorTemperature());
                        mVoltage.setDouble(shoulder.getVoltage());
                        mArbitraryFF.setDouble(shoulder.getArbitraryFF());
                        // mLimitSensor.setBoolean(shoulder.atLimitSensor());

                        if (mOpenLoopToggle.getBoolean(false)) {
                                shoulder.setPercentOutput(DriverOI.getInstance().getArmSpeed());
                        } else if (mPIDToggle.getBoolean(false)) {
                                shoulder.updatePIDController(mkP.getDouble(ShoulderConstants.kP),
                                                mkI.getDouble(ShoulderConstants.kI),
                                                mkD.getDouble(ShoulderConstants.kD),
                                                mkIz.getDouble(ShoulderConstants.kIz), 
                                                mkFF.getDouble(ShoulderConstants.kFF), 0);

                                shoulder.updateShoulderFeedforward(
                                                mkG.getDouble(0.0),
                                                mkV.getDouble(0.0),
                                                mkA.getDouble(0.0));

                                shoulder.setPositionSmartMotion(mPIDSetpoint.getDouble(0.0), SmartMotionArmSpeed.REGULAR);
                                shoulder.setRegularSmartMotionParameters(mSmartMotionAngleTol.getDouble(0.0),
                                                mSmartMotionMinVel.getDouble(0.0), mSmartMotionMaxVel.getDouble(0.0),
                                                mSmartMotionMaxAccel.getDouble(0.0));

                        }

                } catch (NullPointerException e) {
                }
        }

}
