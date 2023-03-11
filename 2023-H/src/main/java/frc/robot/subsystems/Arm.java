package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands.SetLevelThreeConeForwardPose;
import frc.robot.commands.ArmCommands.SetLevelThreeConeInvertedPose;
import frc.robot.commands.ArmCommands.SetLevelThreeCubeForwardPose;
import frc.robot.commands.ArmCommands.SetLevelThreeCubeInvertedPose;
import frc.robot.commands.ArmCommands.SetLevelTwoConePose;
import frc.robot.commands.ArmCommands.SetLevelTwoCubePose;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class Arm extends SubsystemBase {
    private static Arm arm;

    private final Shoulder shoulder;
    private final Wrist wrist;

    public enum ArmState {NONE, HOME, STOWED, TRANSITORY, PRE_SCORE, FLOOR_INTAKE_CUBE_COMPACT,
        FLOOR_INTAKE_CUBE_EXTENDED, FLOOR_INTAKE_CONE_COMPACT, FLOOR_INTAKE_CONE_EXTENDED, SINGLE_SS, DOUBLE_SS_CONE,
        L1, L2_CONE, L2_CUBE, L3_CUBE_FORWARD, L3_CONE_FORWARD, L3_CUBE_INVERTED, L3_CONE_INVERTED};
    
    private ArmState state, goalPose;

    public Arm() {
        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();

        state = ArmState.HOME;
        goalPose = ArmState.NONE;

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
        setShoulderPosition(shoulder.getAngle());
    }

    public void setShoulderPosition(double setPoint){
        shoulder.setPosition(setPoint);
    }

    public void setShoulderPositionSmartMotion(double setPoint, SmartMotionArmSpeed mode){
        shoulder.setPositionSmartMotion(setPoint, mode);
    }

    public void holdWristPosition(){
        setWristPosition(wrist.getAngle());
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

    public boolean isInvertedL3Cone(){
        return state == ArmState.L3_CONE_INVERTED;
    }

    public boolean isInvertedL3Cube(){
        return state == ArmState.L3_CUBE_INVERTED;
    }

    public boolean isInvertedL3(){
        return state == ArmState.L3_CONE_INVERTED || state == ArmState.L3_CUBE_INVERTED;
    }

    public boolean isArmScoringPose(){
        return state == ArmState.L1 || state == ArmState.L2_CONE || state == ArmState.L2_CUBE || state == ArmState.L3_CUBE_FORWARD || state == ArmState.L3_CONE_INVERTED;
    }

    public static Arm getInstance() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

    public boolean isPreScorePose(){
        return state == ArmState.PRE_SCORE;
    }

    public boolean isSingleSSPose(){
        return state == ArmState.SINGLE_SS;
    }

    public ArmState getGoalPose() {
        return goalPose;
    }

    public void setGoalPoseToLevelTwoCone(){
        goalPose = ArmState.L2_CONE;
    }

    public void setGoalPoseToLevelThreeCone(){
        goalPose = ArmState.L3_CONE_INVERTED;
    }

    public void setGoalPoseToLevelTwoCube(){
        goalPose = ArmState.L2_CUBE;
    }

    public void setGoalPoseToLevelThreeCube(){
        goalPose = ArmState.L3_CUBE_FORWARD;
    }

    // Moves from the pre-pose state into a scoring pose.
    public void moveToScoringPose(){
        double poseDeg = Drivetrain.getInstance().getPose().getRotation().getDegrees();
         // For L3 poses, check pose rotation to decide whether the goal pose is still correct.
         // Case 1: The robot is facing away from the goal.
        if(poseDeg > -90 && poseDeg < 90){
            if(goalPose == ArmState.L3_CUBE_FORWARD){
                goalPose = ArmState.L3_CUBE_INVERTED;
            }
            else if(goalPose == ArmState.L3_CONE_FORWARD){
                goalPose = ArmState.L3_CONE_INVERTED;
            }
        }
        // Case 2: The robot is facing toward the goal.
        else{
            if(goalPose == ArmState.L3_CUBE_INVERTED){
                goalPose = ArmState.L3_CUBE_FORWARD;
            }
            else if(goalPose == ArmState.L3_CONE_INVERTED){
                goalPose = ArmState.L3_CONE_FORWARD;
            }    
        }

         // Now execute the goal pose
        if(goalPose == ArmState.L2_CONE){
            CommandScheduler.getInstance().schedule(new SetLevelTwoConePose());
        }
        else if(goalPose == ArmState.L2_CUBE){
            CommandScheduler.getInstance().schedule(new SetLevelTwoCubePose());
        }
        else if(goalPose == ArmState.L3_CUBE_FORWARD){
            CommandScheduler.getInstance().schedule(new SetLevelThreeCubeForwardPose());
        }
        else if(goalPose == ArmState.L3_CUBE_INVERTED){
            CommandScheduler.getInstance().schedule(new SetLevelThreeCubeInvertedPose());
        }
        else if(goalPose == ArmState.L3_CONE_FORWARD){
            CommandScheduler.getInstance().schedule(new SetLevelThreeConeForwardPose());
        }
        else if(goalPose == ArmState.L3_CONE_INVERTED){
            CommandScheduler.getInstance().schedule(new SetLevelThreeConeInvertedPose());
        }

        // The next goal pose for this method to execute is now "none" unless otherwise set by the operator.
        goalPose = ArmState.NONE;
    }

    public void turnOffSmartLimits(){
        wrist.turnOffSmartLimits();
        shoulder.turnOffSmartLimits();
    }

    public void turnOnSmartLimits(){
        wrist.turnOnSmartLimits();
        shoulder.turnOnSmartLimits();
    }

    public boolean isValidEjectPose(){
        return (getState() != ArmState.STOWED && getState() != ArmState.HOME) && Claw.getInstance().hasGamepiece();
    }

    @Override
    public void periodic() {
        shoulder.periodic();
        wrist.periodic();

    }

}