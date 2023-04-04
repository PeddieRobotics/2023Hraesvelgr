package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.InterpolatingTreeMap;
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
        EMPTY, INTAKING_CUBE, INTAKING_CONE, CUBE, CONE, UNKNOWN
    };

    private ClawState state; // Our current best estimation of the intake's state with respect to game pieces

    private boolean gamepieceOperatorOverride;

    private final LimelightFront limelightFront; // Used to help with cone alignment monitoring /calculating offset

    private double gamepieceAlignmentError;
    private double initialAlignmentAnalysisTime;
    private MedianFilter alignmentFilter;
    private boolean monitorNewConeIntake, monitorNewCubeIntake, analyzingAlignment;
    private double currentAlignmentDistance;

    private InterpolatingTreeMap<Double,Double> cubeAlignmentTable, coneL2AlignmentTable, coneL3AlignmentTable;

    private double ejectionTime;
    private boolean justEjectedGamepiece;

    private boolean sawGamepieceDuringAlignmentAnalysis;

    private String currentLLForAutoAlign;

    private boolean normalizingCone;

    public Claw() {
        clawMotor = new CANSparkMax(RobotMap.kClawMotor, MotorType.kBrushless);
        clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
        clawMotor.setIdleMode(IdleMode.kCoast);
        clawMotor.setOpenLoopRampRate(0.1);

        backSensor = new DigitalInput(RobotMap.kClawBackSensor);
        frontSensor = new DigitalInput(RobotMap.kClawFrontSensor);
        useSensors = true;

        state = ClawState.EMPTY;
        ejectionTime = -1;
        justEjectedGamepiece = false;
        gamepieceOperatorOverride = false;

        limelightFront = LimelightFront.getInstance();

        alignmentFilter = new MedianFilter(10); // median filter over last 200 ms
        gamepieceAlignmentError = 0.0;
        initialAlignmentAnalysisTime = 0.0;
        monitorNewConeIntake = false;
        monitorNewCubeIntake = false;
        analyzingAlignment = false;
        currentAlignmentDistance = 0.0;

        currentLLForAutoAlign = "limelight-front";

        currentAverage = new RollingAverage(4);
        monitorCurrent = false;

        normalizingCone = false;

         // Linear interpolation calibrated points for cube alignment
        cubeAlignmentTable = new InterpolatingTreeMap<>();
        cubeAlignmentTable.put(-13.26, -9.93); // Left calibration point
        cubeAlignmentTable.put(8.46, 1.67); // Right calibration point

        // Linear interpolation calibrated points for L2 cone alignment
        coneL2AlignmentTable = new InterpolatingTreeMap<>();
        coneL2AlignmentTable.put(-17.03, -7.52); // Left calibration point
        coneL2AlignmentTable.put(9.93, 2.54); // Right calibration point

        // Linear interpolation calibrated points for L3 cone alignment
        coneL3AlignmentTable = new InterpolatingTreeMap<>();
        coneL3AlignmentTable.put(-17.23, 5.59); // Left calibration point
        coneL3AlignmentTable.put(9.93, -2.54); // Right calibration point
    
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current alignment dist", currentAlignmentDistance);

        // If we've recently intaked a cone or cube and we're in an analysis pose, start looking at it.
        if((monitorNewConeIntake
        && Arm.getInstance().isWristAtAngle(WristConstants.kMonitorConeAlignmentAngle)
        && Arm.getInstance().getState() == ArmState.STOWED) ||
        (monitorNewCubeIntake
        && Arm.getInstance().isWristAtAngle(WristConstants.kMonitorCubeAlignmentAngle)
        && Arm.getInstance().getState() == ArmState.STOWED)){

            if(!analyzingAlignment){
                analyzingAlignment = true;
                initialAlignmentAnalysisTime = Timer.getFPGATimestamp();
            }

            updateGamepieceAlignmentError();
            SmartDashboard.putNumber("Gamepiece alignment error", gamepieceAlignmentError);
        }

        // If we have looked the cone for at least 500 ms, we've gotten enough of a
        // glimpse. This logic is separated from the above so that "finishedAnalyzingAlignment"
        // is guaranteed to run, even if the pose is ordered to change (interrupted).
        if(analyzingAlignment && (monitorNewConeIntake || monitorNewCubeIntake)){
            if (Timer.getFPGATimestamp() - initialAlignmentAnalysisTime > ClawConstants.kMaximumGamepieceMonitorTime) {
                finishedAnalyzingAlignment();
            }
        }

        if(justEjectedGamepiece && ejectionTime - Timer.getFPGATimestamp() > 1){
            justEjectedGamepiece = false;
        }

        if(monitorCurrent){
            currentAverage.add(clawMotor.getOutputCurrent());
        }

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

    public boolean isEitherSensor(){
        return isFrontSensor() || isBackSensor();
    }

    public boolean isBothSensors(){
        return isFrontSensor() && isBackSensor();
    }

    public boolean isNeitherSensor(){
        return !isEitherSensor();
    }

    public boolean hasCone() {
        return state == ClawState.CONE;
    }

    public boolean hasCube() {
        return state == ClawState.CUBE;
    }

    public boolean hasGamepiece() {
        return hasCone() || hasCube() || state == ClawState.UNKNOWN;
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
        if(limelightFront.getTv()){
            sawGamepieceDuringAlignmentAnalysis = true;
            gamepieceAlignmentError = alignmentFilter.calculate(limelightFront.getTxAverage());
        }
    }

    public double convertL2ConeTXToAlignmentError(double tx) {
        return coneL2AlignmentTable.get(tx);
    }

    public double convertL2CubeTXToAlignmentError(double tx) {
        return cubeAlignmentTable.get(tx);
    }

    public double convertL3ConeTXToAlignmentError(double tx) {
        return coneL3AlignmentTable.get(tx);
    }

    public double convertL3CubeTXToAlignmentError(double tx) {
        return cubeAlignmentTable.get(tx);
    }

    public void monitorNewConeIntake() {
        limelightFront.setPipeline(7); // monitor alignment of cone in intake
        limelightFront.resetRollingAverages();
        monitorNewConeIntake = true;
    }

    public void monitorNewCubeIntake() {
        limelightFront.setPipeline(2); // monitor alignment of cube in intake
        limelightFront.resetRollingAverages();
        monitorNewCubeIntake = true;
    }

    public void finishedAnalyzingAlignment(){
        monitorNewConeIntake = false;
        monitorNewCubeIntake = false;
        analyzingAlignment = false;
        sawGamepieceDuringAlignmentAnalysis = false;
        alignmentFilter.reset();
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
        LimelightHelper.setPipelineIndex("limelight-front", 7); // Cone alignment pipeline (color camera)
        LimelightHelper.setPipelineIndex("limelight-back", 0); // April tag pipeline

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

    // Used for detecting a positive cube intake
    public boolean analyzeCurrentForCube() {
        if(currentAverage.getAverage() > 30 && isFrontSensor() && !isBackSensor()){
            return true;
        }
        return false;
    }

    public boolean isNormalizingCone() {
        return normalizingCone;
    }

    public void setNormalizingCone(boolean normalizingCone) {
        this.normalizingCone = normalizingCone;
    }

    public boolean isGamepieceOperatorOverride() {
        return gamepieceOperatorOverride;
    }


    public void setGamepieceOperatorOverride(boolean gamepieceOperatorOverride) {
        this.gamepieceOperatorOverride = gamepieceOperatorOverride;
    }

    public void classifyGamepiece(){
        setGamepieceOperatorOverride(false);
        
        if(isFrontSensor() && !isBackSensor()){
            setState(ClawState.CUBE);
            Blinkin.getInstance().success();
            monitorNewCubeIntake();
            setSpeed(ClawConstants.kCubeHoldSpeed);
        }
        else if(isBackSensor()){
            setState(ClawState.CONE);
            Blinkin.getInstance().success();
            monitorNewConeIntake();
            stopClaw();
        }
        else{
            setState(ClawState.EMPTY); 
            Blinkin.getInstance().returnToRobotState();
            stopClaw();
        }
    }


    // public void checkGamepieceTypeWithVision(){
    //     // Looking for a cone, so check that we saw one in the intake.
    //     if(limelightFront.getPipeline() == 7){
    //         if(!sawGamepieceDuringAlignmentAnalysis){
    //             setState(ClawState.UNKNOWN);
    //             Blinkin.getInstance().returnToRobotState();
    //         }
    //         else{
    //             setState(ClawState.CONE);
    //         }
    //     }
    //     // Looking for a cube, so check that we saw one in the intake.
    //     else if(limelightFront.getPipeline() == 2){
    //         if(!sawGamepieceDuringAlignmentAnalysis){
    //             setState(ClawState.UNKNOWN);
    //             Blinkin.getInstance().returnToRobotState();
    //         }
    //         else{
    //             setState(ClawState.CUBE);
    //         }
    //     }
    // }

    public boolean isSawGamepieceDuringAlignmentAnalysis() {
        return sawGamepieceDuringAlignmentAnalysis;
    }

    public void setSawGamepieceDuringAlignmentAnalysis(boolean sawGamepieceDuringAlignmentAnalysis) {
        this.sawGamepieceDuringAlignmentAnalysis = sawGamepieceDuringAlignmentAnalysis;
    }

    public boolean isAnalyzingAlignment() {
        return analyzingAlignment;
    }

    public void setAnalyzingAlignment(boolean analyzingAlignment) {
        this.analyzingAlignment = analyzingAlignment;
    }

    public double getCurrentAlignmentDistance() {
        return currentAlignmentDistance;
    }

    public void setCurrentAlignmentDistance(double currentAlignmentDistance) {
        this.currentAlignmentDistance = currentAlignmentDistance;
    }

}
