package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands.SetTransitoryPose;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class Arm extends SubsystemBase {
    private static Arm arm;

    private final Shoulder shoulder;
    private final Wrist wrist;

    public enum ArmState {MOVING, HOME, STOWED, TRANSITION, FLOOR_INTAKE_CUBE_COMPACT, FLOOR_INTAKE_CUBE_EXTENDED,
        FLOOR_INTAKE_CONE_COMPACT, FLOOR_INTAKE_CONE_EXTENDED, LL_SEEK, SINGLE_SS, DOUBLE_SS_CONE, DOUBLE_SS_CUBE,
        L1, L2_CONE, L2_CUBE, L3_CUBE_FORWARD, L3_CONE_FORWARD, L3_CUBE_INVERTED, L3_CONE_INVERTED};
    
    private ArmState state;

    public Arm() {
        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        state = ArmState.HOME;
    }

    public ArmState getState() {
        return state;
    }

    public void setState(ArmState state) {
        this.state = state;
    }

    public void setShoulderMode(IdleMode mode){
        shoulder.setMode(mode);
    }

    public void setWristMode(IdleMode mode){
        wrist.setMode(mode);
    }

    public void setWristPercentOutput(double wristSpeed) {
        wrist.setPercentOutput(wristSpeed);
    }

    public double getWristPercentOutput() {
        return wrist.getSpeed();
    }

    public void setShoulderPercentOutput(double shoulderSpeed) {
        shoulder.setPercentOutput(shoulderSpeed);
    }

    public double getShoulderPercentOutput() {
        return shoulder.getPercentOutput();
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

    public void holdShoulderPosition(){
        setShoulderPosition(shoulder.getCurrentPIDSetpointAngle());
    }

    public void setShoulderPosition(double setPoint){
        shoulder.setPosition(setPoint);
    }

    public void setShoulderPositionSmartMotion(double setPoint, SmartMotionArmSpeed mode){
        shoulder.setPositionSmartMotion(setPoint, mode);
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
        return Math.abs(getShoulderPosition() - angle) < ShoulderConstants.kSetpointTolerance;
    }

    public boolean isWristAtAngle(double angle){
        return Math.abs(getWristPosition() - angle) < WristConstants.kSetpointTolerance;
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

    public boolean isShoulderFullyStowed(){
        return (state == ArmState.HOME || state == ArmState.STOWED || state == ArmState.L1);
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

}