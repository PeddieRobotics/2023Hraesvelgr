package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utils.RobotMapH;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm instance;

    private final Shoulder shoulder;
    private final Wrist wrist;

    public Arm() {
        shoulder = Shoulder.getInstance();
        wrist = new Wrist();
    }

    public void setWristSpeed(double wristSpeed) {
        wrist.getMotor().set(wristSpeed);
    }

    public double getWristSpeed() {
        return wrist.getMotor().get();
    }

    public void setShoulderSpeed(double shoulderSpeed) {
        shoulder.setSpeed(shoulderSpeed);
    }

    public double getShoulderSpeed() {
        return shoulder.getSpeed();
    }

    public double getShoulderOutputCurrent() {
        return shoulder.getOutputCurrent();
    }

    public double getShoulderTemperature() {
        return shoulder.getMotorTemperature();
    }

    public double getWristOutputCurrent() {
        return wrist.getMotor().getOutputCurrent();
    }

    public double getWristTemperature() {
        return wrist.getMotor().getMotorTemperature();
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public void putSmartDashboardOverrides() {
        wrist.putSmartDashboardOverrides();
    }

    public void updateFromDashboard() {
        wrist.updateFromDashboard();
    }

    @Override
    public void periodic() {
        wrist.periodic();
        shoulder.periodic();
    }
}