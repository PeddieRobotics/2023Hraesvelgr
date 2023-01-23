package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utils.Motor;
import frc.robot.utils.RobotMapH;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm instance;

    private class ShoulderMotor extends Motor {
        // this is where you "override" constants (don't do the @Override thing)
        // TODO: test if the override thing works

        @Override
        protected int getMotorCanId() {
            return 0;
        }

        @Override
        protected String getPrefixedString(String baseString) {
            return "Shoulder " + baseString;
        }
    }

    private Motor shoulder, wrist; // TODO: elbow, only extends once at the begining of the game

    private CANSparkMax shoulderMotor, wristMotor;

    public Arm() {

    }

    public void setWristSpeed(double wristSpeed) {
        wristMotor.set(wristSpeed);
    }

    public double getWristSpeed() {
        return wristMotor.get();
    }

    public void setShoulderSpeed(double shoulderSpeed) {
        shoulderMotor.set(shoulderSpeed);
    }

    public double getShoulderSpeed() {
        return shoulderMotor.get();
    }

    public double getShoulderMotorOutputCurrent() {
        return shoulderMotor.getOutputCurrent();
    }

    public double getShoulderMotorTemperature() {
        return shoulderMotor.getMotorTemperature();
    }

    public double getWristOutputCurrent() {
        return wristMotor.getOutputCurrent();
    }

    public double getWristMotorTemperature() {
        return wristMotor.getMotorTemperature();
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public void putShuffleboard() {
        SmartDashboard.putNumber(RobotMapH.kShoulderMotor + " Shoulder RPM", getShoulderSpeed());
        SmartDashboard.putNumber(RobotMapH.kWristMotor + " Wrist RPM", getWristSpeed());
    }
}