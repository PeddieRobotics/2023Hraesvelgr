package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Shoulder;
import frc.robot.utils.OI;
import frc.robot.utils.Constants.ShoulderConstants;

public class ShoulderTab extends ShuffleboardTabBase {
    private Shoulder shoulder = Shoulder.getInstance();

    private GenericEntry mSpeed, mAngle, mCurrent, mTemp, mArbitraryFF, mOpenLoopToggle, mPIDToggle,
    mkG, mkV, mkA, mkP, mkI, mkD, mkIz, mPIDSetpoint, mSmartMotionAngleTol, mSmartMotionMinVel, mSmartMotionMaxVel, mSmartMotionMaxAccel;

public ShoulderTab(){
}

public void createEntries() {
        tab = Shuffleboard.getTab("Shoulder");

        try{
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

                mSmartMotionAngleTol = tab.add("S.M. Setpoint Tol", ShoulderConstants.kSmartMotionSetpointTol)
                .getEntry();
                mSmartMotionMinVel = tab.add("S.M. Min Vel", ShoulderConstants.kSmartMotionMinVel)
                .getEntry();
                mSmartMotionMaxVel = tab.add("S.M. Max Vel", ShoulderConstants.kSmartMotionMaxVel)
                .getEntry();
                mSmartMotionMaxAccel = tab.add("S.M. Max Accel", ShoulderConstants.kSmartMotionMaxAccel)
                .getEntry();
        } catch(IllegalArgumentException e){
        }

}

@Override
public void update() {
        try{
                mSpeed.setDouble(shoulder.getVelocity());
                mAngle.setDouble(shoulder.getPosition());
                mCurrent.setDouble(shoulder.getOutputCurrent());
                mTemp.setDouble(shoulder.getMotorTemperature());
                mArbitraryFF.setDouble(shoulder.getArbitraryFF());

                if (mOpenLoopToggle.getBoolean(false)) {
                        shoulder.setPercentOutput(OI.getInstance().getArmSpeed());
                }
                else if (mPIDToggle.getBoolean(false)) {
                        shoulder.setPIDController(mkP.getDouble(ShoulderConstants.kP),
                                mkI.getDouble(ShoulderConstants.kI),
                                mkD.getDouble(ShoulderConstants.kD),
                                mkIz.getDouble(ShoulderConstants.kIz));

                        shoulder.setShoulderFeedforward(
                                mkG.getDouble(0.0),
                                mkV.getDouble(0.0),
                                mkA.getDouble(0.0));

                shoulder.setPositionSmartMotion(mPIDSetpoint.getDouble(0.0));
                shoulder.setSmartMotionParameters(mSmartMotionAngleTol.getDouble(0.0),
                mSmartMotionMinVel.getDouble(0.0), mSmartMotionMaxVel.getDouble(0.0),
                mSmartMotionMaxAccel.getDouble(0.0));

        }

        } catch(NullPointerException e){
        }
}

}
