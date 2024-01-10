package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands.SetLevelThreeConeForwardPose;
import frc.robot.commands.ArmCommands.SetLevelThreeConeInvertedPose;
import frc.robot.commands.ArmCommands.SetLevelThreeCubeForwardPose;
import frc.robot.commands.ArmCommands.SetLevelTwoConePose;
import frc.robot.commands.ArmCommands.SetLevelTwoCubePose;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class Arm extends SubsystemBase {
    private static Arm arm;

    private final Shoulder shoulder;
    private final Wrist wrist;

    // Some of these states are currently unused but possible.
    public enum ArmState {NONE, HOME, STOWED, TRANSITORY, PRE_SCORE, FLOOR_INTAKE_CUBE_COMPACT,
        FLOOR_INTAKE_CUBE_EXTENDED, FLOOR_INTAKE_CONE_COMPACT, FLOOR_INTAKE_CONE_EXTENDED, SINGLE_SS_CUBE,
        SINGLE_SS_CONE, DOUBLE_SS_CUBE, DOUBLE_SS_CONE,
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
        setWristPosition(wrist.getPosition());
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

    public boolean isWristLessThanAngle(double angle){
        return getWristPosition() < angle;
    }

    public boolean isWristGreaterThanAngle(double angle){
        return getWristPosition() > angle;
    }

    public boolean isUnsafeSnapPose(){
        return state == ArmState.DOUBLE_SS_CONE || state == ArmState.DOUBLE_SS_CUBE || state == ArmState.FLOOR_INTAKE_CONE_EXTENDED ||
        state == ArmState.L3_CONE_INVERTED || state == ArmState.L3_CUBE_INVERTED;
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
        return state == ArmState.L1 || state == ArmState.L2_CONE || state == ArmState.L2_CUBE ||
        state == ArmState.L3_CUBE_FORWARD || state == ArmState.L3_CONE_INVERTED ||
        state == ArmState.L3_CUBE_INVERTED || state == ArmState.L3_CONE_FORWARD;
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

    public boolean isScoringAutoAlignPose(){
        return isPreScorePose() || isArmScoringPose();
    }

    public boolean isAutoAlignValid(){
        boolean headingIsCorrect = false;
        if(Drivetrain.getInstance().getFlipped()){
            if(goalPose == ArmState.L3_CONE_INVERTED || goalPose == ArmState.L3_CUBE_INVERTED || state == ArmState.L3_CONE_INVERTED || state == ArmState.L3_CUBE_INVERTED){
                headingIsCorrect = Math.abs(Drivetrain.getInstance().getHeading()) < 25.0;
            }
            else{
                headingIsCorrect = Math.abs(Drivetrain.getInstance().getHeading()) > 100.0;
            }
        }
        else{
            if(goalPose == ArmState.L3_CONE_INVERTED || goalPose == ArmState.L3_CUBE_INVERTED || state == ArmState.L3_CONE_INVERTED || state == ArmState.L3_CUBE_INVERTED){
                headingIsCorrect = Math.abs(Drivetrain.getInstance().getHeading()) > 155.0;
            }
            else{
                headingIsCorrect = Math.abs(Drivetrain.getInstance().getHeading()) < 80.0;
            }            
        }
        return isScoringAutoAlignPose() && headingIsCorrect;
    }

    public boolean isSingleSSPose(){
        return state == ArmState.SINGLE_SS_CUBE || state == ArmState.SINGLE_SS_CONE;
    }

    public boolean isDoubleSSPose(){
        return state == ArmState.DOUBLE_SS_CUBE || state == ArmState.DOUBLE_SS_CONE;
    }

    public boolean isL1Pose(){
        return state == ArmState.L1;
    }

    public boolean isL2L3ForwardScoringPose(){
        return state == ArmState.L2_CONE || state == ArmState.L2_CUBE || state == ArmState.L3_CUBE_FORWARD || state == ArmState.L3_CONE_FORWARD;
    }

    public ArmState getGoalPose() {
        return goalPose;
    }

    public void setGoalPose(ArmState pose){
        goalPose = pose;
    }

    public void setGoalPoseToLevelTwoCone(){
        goalPose = ArmState.L2_CONE;
    }

    public void setGoalPoseToLevelThreeConeForward(){
        goalPose = ArmState.L3_CONE_FORWARD;
    }

    public void setGoalPoseToLevelThreeConeInverted(){
        goalPose = ArmState.L3_CONE_INVERTED;
    }

    public void setGoalPoseToLevelTwoCube(){
        goalPose = ArmState.L2_CUBE;
    }

    public void setGoalPoseToLevelThreeCubeForward(){
        goalPose = ArmState.L3_CUBE_FORWARD;
    }

    public void setGoalPoseToLevelThreeCubeInverted(){
        goalPose = ArmState.L3_CUBE_INVERTED;
    }

    // Moves from the pre-pose state into a scoring pose.
    public void moveToScoringPose(){
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
        // else if(goalPose == ArmState.L3_CUBE_INVERTED){
        //     CommandScheduler.getInstance().schedule(new SetLevelThreeCubeInvertedPose());
        // }
        else if(goalPose == ArmState.L3_CONE_FORWARD){
            CommandScheduler.getInstance().schedule(new SetLevelThreeConeForwardPose());
        }
        else if(goalPose == ArmState.L3_CONE_INVERTED){
            CommandScheduler.getInstance().schedule(new SetLevelThreeConeInvertedPose());
        }

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
        return (state != ArmState.STOWED && state != ArmState.HOME) && Claw.getInstance().hasGamepiece();
    }

    public boolean canNewIntake(){
        return isShoulderBelowAngle(-60) && isWristLessThanAngle(80);
    }
    
    @Override
    public void periodic() {
        // Do not let the shoulder pull 30+ amps for more than 20 seconds.
        // Shut off the motor if so.
        if(shoulder.getMotorTemperature() > 50.0){
            shoulder.setPercentOutput(0);
        }

        shoulder.periodic();
        wrist.periodic();

    }

}