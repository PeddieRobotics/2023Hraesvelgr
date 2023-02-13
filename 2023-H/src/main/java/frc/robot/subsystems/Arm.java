package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm arm;

    private final Shoulder shoulder;
    private final Wrist wrist;

    public Arm() {
        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    public void setShoulderMode(IdleMode mode){
        shoulder.setMode(mode);
    }

    public void setWristSpeed(double wristSpeed) {
        wrist.setPercentOutput(wristSpeed);
    }

    public double getWristSpeed() {
        return wrist.getSpeed();
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
        return wrist.getOutputCurrent();
    }

    public void setShoulderPosition(double setPoint){
        shoulder.setPosition(setPoint);
    }

    public void setWristPosition(double setPoint){
        wrist.setPosition(setPoint);
    }

    public void stopShoulder(){
        shoulder.stopShoulder();
    }

    public void stopWrist(){
        wrist.stopWrist();
    }

    public double getWristTemperature() {
        return wrist.getMotorTemperature();
    }

    public static Arm getInstance() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

    @Override
    public void periodic() {
        shoulder.periodic();
        wrist.periodic();
    }

    public void testPeriodic(){
        shoulder.testPeriodic();
        wrist.testPeriodic();
    }
}