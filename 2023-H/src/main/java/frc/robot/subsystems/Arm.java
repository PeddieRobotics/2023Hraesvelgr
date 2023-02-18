package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

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

    public void setWristMode(IdleMode mode){
        wrist.setMode(mode);
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

    public double getShoulderPosition() {
        return shoulder.getPosition();
    }

    public double getWristPosition() {
        return wrist.getPosition();
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

    public boolean isShoulderAtAngle(double angle){
        return Math.abs(getShoulderPosition() - angle) < ShoulderConstants.kShoulderSetpointTolerance;
    }

    public boolean isWristAtAngle(double angle){
        return Math.abs(getWristPosition() - angle) < WristConstants.kWristSetpointTolerance;
    }

    public boolean isShoulderAboveAngle(double angle){
        return getShoulderPosition() > angle;
    }

    public boolean isShoulderBelowAngle(double angle){
        return getShoulderPosition() < angle;
    }

    public boolean isWristAboveAngle(double angle){
        return getWristPosition() > angle;
    }

    public boolean isWristBelowAngle(double angle){
        return getWristPosition() < angle;
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