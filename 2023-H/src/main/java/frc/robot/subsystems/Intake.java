package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMapH;

public class Intake extends SubsystemBase {
    private static Intake instance;

    private static final int MAX_INTAKE_CURRENT = 10; // TODO: change value

    private CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(RobotMapH.MOTOR_INTAKE, MotorType.kBrushed); // TODO: no idea what brushed means
        
        intakeMotor.setSmartCurrentLimit(MAX_INTAKE_CURRENT);
    }

    public void setIntakeSpeed(double intakeSpeed) {
        intakeMotor.set(intakeSpeed);
    }

    public double getIntakeSpeed() {
        return intakeMotor.get();
    }

    public void stopIntake() {
        setIntakeSpeed(0);
    }

    public double getIntakeOutputCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    public double getIntakeMotorTemperature() {
        return intakeMotor.getMotorTemperature();
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    public void putShuffleboard() {
        SmartDashboard.putNumber(RobotMapH.MOTOR_INTAKE + " Intake RPM", getIntakeSpeed());
    }

    public void updateIntakeFromDashboard(){
        double desiredIntakeSpeed = SmartDashboard.getNumber(RobotMapH.MOTOR_INTAKE + " Intake RPM", getIntakeSpeed());
        if (desiredIntakeSpeed!=getIntakeSpeed()) {
            setIntakeSpeed(desiredIntakeSpeed);
        }
    }
}