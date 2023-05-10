package frc.robot.Shuffleboard.tabs;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.SmartMotionWristSpeed;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.WristConstants;

public class WristTab extends ShuffleboardTabBase {
    private Wrist wrist = Wrist.getInstance();

    private GenericEntry mVelocity, mAngle, mCurrent, mTemp, mVoltage, mArbitraryFF, mOpenLoopToggle, mPIDToggle,
            mkG, mkV, mkA, mkP, mkI, mkD, mkIz, mkFF, mPIDSetpoint, mSmartMotionAngleTol, mSmartMotionMinVel,
            mSmartMotionMaxVel, mSmartMotionMaxAccel, mLimitSensor;

    public WristTab() {
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Wrist");

        try {
            mVelocity = tab.add("Speed", 0.0)
                    .getEntry();
            mAngle = tab.add("Angle", 0.0)
                    .getEntry();
            mCurrent = tab.add("Current", 0.0)
                    .getEntry();
            mTemp = tab.add("Temperature", 0.0)
                    .getEntry();
            mVoltage = tab.add("Voltage", 0.0)
                    .getEntry();
        // mLimitSensor = tab.add("Limit sensor", false)
        //         .getEntry();
            mOpenLoopToggle = tab.add("Open Loop Toggle", false)
                    .withWidget(BuiltInWidgets.kToggleButton)
                    .getEntry();
            mPIDToggle = tab.add("PID Toggle", false)
                    .withWidget(BuiltInWidgets.kToggleButton)
                    .getEntry();
            mPIDSetpoint = tab.add("PID Setpoint", WristConstants.kHomeAngle)
                    .getEntry();
            mArbitraryFF = tab.add("Arb FF", 0.0)
                    .getEntry();
            mkG = tab.add("kG", WristConstants.kGVolts)
                    .getEntry();
            mkV = tab.add("kV", WristConstants.kVVoltSecondPerRad)
                    .getEntry();
            mkA = tab.add("kA", WristConstants.kAVoltSecondSquaredPerRad)
                    .getEntry();
            mkP = tab.add("kP", WristConstants.kPositionP)
                    .getEntry();
            mkI = tab.add("kI", WristConstants.kPositionI)
                    .getEntry();
            mkIz = tab.add("IZone", WristConstants.kPositionIz)
                    .getEntry();
            mkD = tab.add("kD", WristConstants.kPositionD)
                    .getEntry();
                mkFF = tab.add("kFF", WristConstants.kPositionFF).getEntry();

                mSmartMotionAngleTol = tab.add("mSmartMotionAngleTol", WristConstants.kSmartMotionRegularSetpointTol).getEntry();
                mSmartMotionMinVel = tab.add("mSmartMotionMinVel", WristConstants.kSmartMotionRegularMinVel).getEntry();
                mSmartMotionMaxVel = tab.add("mSmartMotionMaxVel", WristConstants.kSmartMotionRegularMaxVel).getEntry();
                mSmartMotionMaxAccel = tab.add("mSmartMotionMaxAccel", WristConstants.kSmartMotionRegularMaxAccel).getEntry();

        } catch (IllegalArgumentException e) {
        }

    }

    @Override
    public void update() {
        try {
            mVelocity.setDouble(wrist.getVelocity());
            mAngle.setDouble(wrist.getPosition());
            mCurrent.setDouble(wrist.getOutputCurrent());
            mTemp.setDouble(wrist.getMotorTemperature());
            mVoltage.setDouble(wrist.getVoltage());
            mArbitraryFF.setDouble(wrist.getArbitraryFF());
        //     mLimitSensor.setBoolean(wrist.atLimitSensor());

            if (mOpenLoopToggle.getBoolean(false)) {
                wrist.setPercentOutput(DriverOI.getInstance().getArmSpeed());
            } else if (mPIDToggle.getBoolean(false)) {
                wrist.updatePIDController(
                        mkP.getDouble(WristConstants.kPositionP),
                        mkI.getDouble(WristConstants.kPositionI),
                        mkD.getDouble(WristConstants.kPositionD),
                        mkIz.getDouble(WristConstants.kPositionIz), 
                        mkFF.getDouble(WristConstants.kPositionFF), 0);
                        wrist.updateWristFeedforward(
                                        mkG.getDouble(0.0),
                                        mkV.getDouble(0.0),
                                        mkA.getDouble(0.0));
                        wrist.setPosition(mPIDSetpoint.getDouble(0.0));
                        // wrist.setVelocity(mPIDSetpoint.getDouble(0.0));
                        // wrist.setPositionSmartMotion(mPIDSetpoint.getDouble(0.0), SmartMotionWristSpeed.REGULAR);
                        wrist.setRegularSmartMotionParameters(mSmartMotionAngleTol.getDouble(0.0),
                                        mSmartMotionMinVel.getDouble(0.0), mSmartMotionMaxVel.getDouble(0.0),
                                        mSmartMotionMaxAccel.getDouble(0.0));


            }
        } catch (NullPointerException e) {
                SmartDashboard.putString("Catch", "NULL POINTER");
        }
    }

}
