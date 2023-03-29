package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
        EMPTY, INTAKING_CUBE, INTAKING_CONE, CUBE, CONE
    };

    private ClawState state; // Our current best estimation of the intake's state with respect to game pieces

    private boolean gamepieceOperatorOverride;

    private final LimelightFront limelightFront; // Used to help with cone alignment monitoring /calculating offset

    private double gamepieceAlignmentError;
    private boolean monitorNewConeIntake, monitorNewCubeIntake;
    private InterpolatingTreeMap<Double,Double> cubeAlignmentTable, coneL2AlignmentTable, coneL3AlignmentTable;

    private double ejectionTime;
    private boolean justEjectedGamepiece;

    private int newGamepieceCounter;

    private String currentLLForAutoAlign;

    private boolean normalizingCone;

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
        gamepieceOperatorOverride = false;

        limelightFront = LimelightFront.getInstance();
        gamepieceAlignmentError = 0.0;
        monitorNewConeIntake = false;
        monitorNewCubeIntake = false;

        newGamepieceCounter = 0;
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
        
        // Old pre-competition values
        // coneL2AlignmentTable.put(-17.228, -7.79); // Left calibration point
        // coneL2AlignmentTable.put(-4.946, -3.53); // Center calibration point
        // coneL2AlignmentTable.put(11.399, 2.04); // Right calibration point

        // coneL3AlignmentTable.put(-17.583, 9.03); // Left calibration point
        // coneL3AlignmentTable.put(-5.442, 3.57); // Center calibration point
        // coneL3AlignmentTable.put(8.162, -2.7); // Right calibration point
    }

    @Override
    public void periodic() {
        if (useSensors && !gamepieceOperatorOverride) {
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
        if (monitorNewConeIntake && !isNormalizingCone() && limelightFront.hasTarget() && Arm.getInstance().isWristAtAngle(WristConstants.kMonitorConeAlignmentAngle)) {
            newGamepieceCounter++;
            updateGamepieceAlignmentError();

            // If we have looked the cone for at least 100 ms, we've gotten enough of a
            // glimpse.
            if (newGamepieceCounter > 4) {
                monitorNewConeIntake = false; // Stop looking at cone alignment
                newGamepieceCounter = 0;
                returnLimelightToDefaultState();
            }
        }

        // If we've recently intaked a cube, and the arm is oriented such that we could
        // see the cube, start looking at it.
        if (monitorNewCubeIntake && limelightFront.hasTarget() && Arm.getInstance().isWristAtAngle(WristConstants.kMonitorCubeAlignmentAngle)) {
            newGamepieceCounter++;
            updateGamepieceAlignmentError();

            // If we have looked the cube for at least 100 ms, we've gotten enough of a
            // glimpse.
            if (newGamepieceCounter > 4) {
                monitorNewCubeIntake = false; // Stop looking at cube alignment
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
        double sum = gamepieceAlignmentError * (newGamepieceCounter - 1); // recover the previous sum
        gamepieceAlignmentError = (sum + limelightFront.getTxAverage())/newGamepieceCounter; // average any observations so far
        SmartDashboard.putNumber("raw gamepiece tx error", gamepieceAlignmentError);
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
        limelightFront.setPipeline(4); // monitor alignment of cube in intake
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
        LimelightHelper.setPipelineIndex("limelight-front", 7); // Read alignment of cones in intake for auto-align
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
        if(currentAverage.getAverage() > 20 && hasCube() && !hasCone()){
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

}
