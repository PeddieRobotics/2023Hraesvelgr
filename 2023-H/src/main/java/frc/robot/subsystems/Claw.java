package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.RobotMap;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Constants.WristConstants;

public class Claw extends SubsystemBase {
    private static Claw instance;
    private CANSparkMax clawMotor;
    private DigitalInput backSensor, frontSensor;
    private boolean useSensors;

    private RollingAverage currentAverage;
    private boolean monitorCurrent;

    // Keep in mind when using ClawState that the floor cube intake picks up tipped over cones,
    // so "INTAKING_CUBE" state may encompass this possibility.
    public enum ClawState {
        EMPTY, INTAKING_CUBE, INTAKING_CONE, CUBE, CONE
    };

    private ClawState state; // Our current best estimation of the intake's state with respect to game pieces

    private final LimelightFront limelightFront; // Used to help with cone alignment monitoring /calculating offset

    private double gamepieceAlignmentError;
    private boolean monitorNewConeIntake, monitorNewCubeIntake;
    
    private double ejectionTime;
    private boolean justEjectedGamepiece;

    private int newGamepieceCounter;

    private String currentLLForAutoAlign;

    public Claw() {
        clawMotor = new CANSparkMax(RobotMap.kClawMotor, MotorType.kBrushless);
        clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
        clawMotor.setIdleMode(IdleMode.kCoast);

        backSensor = new DigitalInput(RobotMap.kClawBackSensor);
        frontSensor = new DigitalInput(RobotMap.kClawFrontSensor);
        useSensors = true;

        state = ClawState.EMPTY;
        ejectionTime = -1;
        justEjectedGamepiece = false;

        limelightFront = LimelightFront.getInstance();
        gamepieceAlignmentError = 0.0;
        monitorNewConeIntake = false;
        monitorNewCubeIntake = false;

        newGamepieceCounter = 0;
        currentLLForAutoAlign = "limelight-front";

        currentAverage = new RollingAverage(4);
        monitorCurrent = false;
        
    }

    @Override
    public void periodic() {
        if (useSensors) {
            if (isFrontSensor() && isBackSensor()) {
                state = ClawState.CONE;
            } else if (isFrontSensor() && !isBackSensor()) {
                state = ClawState.CUBE;
            } else if (state != ClawState.INTAKING_CONE && state != ClawState.INTAKING_CUBE && (!isFrontSensor() && !isBackSensor())) {
                state = ClawState.EMPTY;
                if(!justEjectedGamepiece){
                    Blinkin.getInstance().emptyCheckForFailure();
                }
            }
        }

        // If we've recently intaked a cone, and the arm is oriented such that we could
        // see the cone, start looking at it.
        if (monitorNewConeIntake && limelightFront.hasTarget() && Arm.getInstance().isWristAtAngle(WristConstants.kMonitorConeAlignmentAngle)) {
            updateGamepieceAlignmentError();
            newGamepieceCounter++;

            // If we have looked the cone for at least 100 ms, we've gotten enough of a
            // glimpse.
            if (newGamepieceCounter > 4) {
                monitorNewConeIntake = false; // Stop looking at cone alignment
                newGamepieceCounter = 0;
                returnLimelightToDefaultState();
            }
        }

        // If we've recently intaked a cube, and the arm is oriented such that we could
        // see the cone, start looking at it.
        if (monitorNewCubeIntake && limelightFront.hasTarget() && Arm.getInstance().isWristAtAngle(WristConstants.kMonitorCubeAlignmentAngle)) {
            updateGamepieceAlignmentError();
            newGamepieceCounter++;

            // If we have looked the cube for at least 100 ms, we've gotten enough of a
            // glimpse.
            if (newGamepieceCounter > 4) {
                monitorNewCubeIntake = false; // Stop looking at cone alignment
                newGamepieceCounter = 0;
                returnLimelightToDefaultState();
            }
        }


        if(justEjectedGamepiece && ejectionTime - Timer.getFPGATimestamp() > 1){
            justEjectedGamepiece = false;
        }

        if(monitorCurrent){
            currentAverage.add(clawMotor.getOutputCurrent());
        }

        SmartDashboard.putNumber("intake current avg", currentAverage.getAverage());

    }

    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }
        return instance;
    }

    public ClawState getState() {
        return state;
    }

    public void setState(ClawState state) {
        this.state = state;
    }

    public void setSpeed(double clawSpeed) {
        clawMotor.set(clawSpeed);
    }

    public boolean isBackSensor() {
        return !backSensor.get();
    }

    public boolean isFrontSensor() {
        return !frontSensor.get();
    }

    public boolean hasCone() {
        return state == ClawState.CONE;
    }

    public boolean hasCube() {
        return state == ClawState.CUBE;
    }

    public boolean hasGamepiece() {
        return hasCone() || hasCube();
    }

    public double getClawSpeed() {
        return clawMotor.get();
    }

    public void stopClaw() {
        clawMotor.set(0);
    }

    public void intakeCube() {
        if(Arm.getInstance().getState() == ArmState.SINGLE_SS_CUBE){
            setSpeed(ClawConstants.kCubeSingleSSIntakeSpeed);
        } else if(Arm.getInstance().getState() == ArmState.SINGLE_SS_CONE){
            setSpeed(ClawConstants.kConeSingleSSIntakeSpeed);
        }
        else{
            setSpeed(ClawConstants.kCubeIntakeSpeed);
        }
    }

    public void intakeCone() {
        setSpeed(ClawConstants.kConeIntakeSpeed);
    }

    public void outtakeCube() {
        ArmState armState = Arm.getInstance().getState();
        if(armState == ArmState.L1){
            setSpeed(ClawConstants.kCubeL1OuttakeSpeed);
        }
        else if(armState == ArmState.L2_CONE){
            setSpeed(ClawConstants.kCubeL2OuttakeSpeed);
        }
        else if(armState == ArmState.L3_CUBE_INVERTED){
            setSpeed(ClawConstants.kCubeL3InvertedOuttakeSpeed);
        }
        else if(armState == ArmState.L3_CUBE_FORWARD){
            setSpeed(ClawConstants.kCubeL3ForwardOuttakeSpeed);
        }
        else{
            setSpeed(ClawConstants.kConeL1OuttakeSpeed);
        }
    }

    public void outtakeCone() {
        ArmState armState = Arm.getInstance().getState();
        if(armState == ArmState.L1){
            setSpeed(ClawConstants.kConeL1OuttakeSpeed);
        }
        else if(armState == ArmState.L2_CONE){
            setSpeed(ClawConstants.kConeL2OuttakeSpeed);
        }
        else if(armState == ArmState.L3_CONE_INVERTED){
            setSpeed(ClawConstants.kConeL3InvertedOuttakeSpeed);
        }
        else if(armState == ArmState.L3_CONE_FORWARD){
            setSpeed(ClawConstants.kConeL3ForwardOuttakeSpeed);
        }
        else{
            setSpeed(ClawConstants.kConeL1OuttakeSpeed);
        }

    }

    public double getMotorTemperature() {
        return clawMotor.getMotorTemperature();
    }

    public double getSpeed() {
        return clawMotor.get();
    }

    public double getOutputCurrent() {
        return clawMotor.getOutputCurrent();
    }

    public double getVoltage() {
        return clawMotor.getAppliedOutput() * 100;
    }

    public boolean isUseSensors() {
        return useSensors;
    }

    public void setUseSensors(boolean useSensors) {
        this.useSensors = useSensors;
    }

    public double getGamepieceAlignmentError() {
        return gamepieceAlignmentError;
    }

    public void setGamepieceAlignmentError(double coneAlignmentError) {
        this.gamepieceAlignmentError = coneAlignmentError;
    }

    public void updateGamepieceAlignmentError() {
        double tx = limelightFront.getTxAverage();
        SmartDashboard.putNumber("raw gamepiece tx error", tx);
        if(state == ClawState.CUBE) {
            gamepieceAlignmentError = convertCubeTXToAlignmentError(tx);
        } else if(state == ClawState.CONE){
            gamepieceAlignmentError = convertConeTXToAlignmentError(tx);
        }
        else{
            gamepieceAlignmentError = 0.0;
        }
        SmartDashboard.putNumber("converted gamepiece tx error", gamepieceAlignmentError);

    }

    public double convertConeTXToAlignmentError(double tx) {
        return tx / 3; // Use y=1/3 x as a simple linear regression (based on some quick empirical
                         // data), where y is robot tx to goal, x is cone tx compared to center of
                         // intake.
    }

    public double convertCubeTXToAlignmentError(double tx) {
        return tx / 3; // Use y=1/3 x as a simple linear regression (based on some quick empirical
                         // data), where y is robot tx to goal, x is cube tx compared to center of
                         // intake.
    }

    public void monitorNewConeIntake() {
        limelightFront.setPipeline(7);
        limelightFront.resetRollingAverages();
        monitorNewConeIntake = true;
    }

    public void monitorNewCubeIntake() {
        limelightFront.setPipeline(4);
        limelightFront.resetRollingAverages();
        monitorNewCubeIntake = true;
    }

    public boolean isMonitorNewConeIntake(){
        return monitorNewConeIntake;
    }

    public void setMonitorNewConeIntake(boolean monitorNewConeIntake) {
        this.monitorNewConeIntake = monitorNewConeIntake;
    }

    public boolean isMonitorNewCubeIntake() {
        return monitorNewCubeIntake;
    }

    public void setMonitorNewCubeIntake(boolean monitorNewCubeIntake) {
        this.monitorNewCubeIntake = monitorNewCubeIntake;
    }

    public void resetGamepieceAlignmentError(){
        gamepieceAlignmentError = 0;
    }

    public double getEjectionTime() {
        return ejectionTime;
    }

    public void setEjectionTime(double ejectionTime) {
        this.ejectionTime = ejectionTime;
    }

    public boolean isJustEjectedGamepiece() {
        return justEjectedGamepiece;
    }

    public void setJustEjectedGamepiece(boolean justEjectedGamepiece) {
        this.justEjectedGamepiece = justEjectedGamepiece;
    }

    public void prepareLimelightForScoring() {
        switch (Arm.getInstance().getGoalPose()) {
            case L3_CUBE_INVERTED:
                currentLLForAutoAlign = "limelight-back";
                break;
            case L3_CONE_INVERTED:
                currentLLForAutoAlign = "limelight-back";
                break;
            default:
                currentLLForAutoAlign = "limelight-front";
        }

        if (hasCone()) {
            LimelightHelper.setPipelineIndex(currentLLForAutoAlign, 6); // Retroreflective tape pipeline
        } else {
            LimelightHelper.setPipelineIndex(currentLLForAutoAlign, 0); // April tag pipeline
        }

    }

    public void returnLimelightToDefaultState(){
        LimelightHelper.setPipelineIndex("limelight-front", 7); // Retroreflective tape pipeline
        LimelightHelper.setPipelineIndex("limelight-back", 0); // Retroreflective tape pipeline

    }

    public String getCurrentLLForAutoAlign(){
        return currentLLForAutoAlign;
    }

    public void startMonitoringCurrent() {
        monitorCurrent = true;
    }

    public void stopMonitoringCurrent() {
        monitorCurrent = false;
        currentAverage.clear();
    }

    public double getCurrentAverage(){
        return currentAverage.getAverage();
    }

    public boolean analyzeCurrentForCube() {
        if(currentAverage.getAverage() > 20 && hasCube() && !hasCone()){
            return true;
        }
        return false;
    }
}
