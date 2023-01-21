package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm instance;

    // TODO: elbow, only extends once at the begining of the game

    private CANSparkMax shoulderMotor, wristMotor, intakeMotor;

    // TODO: constructor

    public void setIntakeSpeed(double intakeSpeed) {
        intakeMotor.set(intakeSpeed);
    }

    public double getIntakeSpeed() {
        return intakeMotor.get();
    }

    public void stopIntake() {
        // TODO: add shoulder lifts and stuff
        intakeMotor.set(0);
    }

    public double getIntakeOutputCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    public double getIntakeMotorTemperature() {
        return intakeMotor.getMotorTemperature();
    }

    public static Arm getInstance() {
        if(instance==null) {
            instance=new Arm();
        }
        return instance;
    }
}