package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.utils.RobotMap;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.WristConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist {
    private static Wrist wrist;

    private CANSparkMax wristMotor;

    private DigitalInput limitSensor;

    private RollingAverage currentAverage;
    private boolean monitorCurrent;

    private SparkPIDController pidController;

    private ArmFeedforward wristFeedforward;

    private double kPositionP, kPositionI, kPositionD, kPositionIz, kP, kI, kD, kIz, kFF, kG, kV, kA, arbitraryFF,
    kSmartMotionRegularSetpointTol, kSmartMotionRegularMinVel,
    kSmartMotionRegularMaxVel, kSmartMotionRegularMaxAccel, kSmartMotionSlowSetpointTol, kSmartMotionSlowMinVel,
    kSmartMotionSlowMaxVel, kSmartMotionSlowMaxAccel, kSmartMotionFastSetpointTol, kSmartMotionFastMinVel,
    kSmartMotionFastMaxVel, kSmartMotionFastMaxAccel;;

    // Angles (poses) start here
    private double kHomeAngle = WristConstants.kHomeAngle;
    private double kStowedAngle = WristConstants.kStowedAngle;
    private double kPreScoreAngle = WristConstants.kPreScoreAngle;

    private double kL1Angle = WristConstants.kL1Angle;

    // Shoulder is not fully extended out
    private double kCompactFloorConeAngle = WristConstants.kCompactFloorConeAngle;
    private double kCompactFloorCubeAngle = WristConstants.kCompactFloorCubeAngle;

    // Shoulder is fully extended out
    private double kExtendedFloorConeAngle = WristConstants.kExtendedFloorConeAngle;
    private double kExtendedFloorCubeAngle = WristConstants.kExtendedFloorCubeAngle;

    private double kL2ConeAngle = WristConstants.kL2ConeAngle;
    private double kL2CubeAngle = WristConstants.kL2CubeAngle;

    private double kL3CubeForwardAngle = WristConstants.kL3CubeForwardAngle;
    private double kL3CubeInvertedAngle = WristConstants.kL3CubeInvertedAngle;

    private double kL3ConeForwardAngle = WristConstants.kL3ConeForwardAngle;
    private double kL3ConeInvertedAngle = WristConstants.kL3ConeInvertedAngle;

    private double kDoubleSSConeAngle = WristConstants.kDoubleSSConeAngle;
    private double kSingleSSConeAngle = WristConstants.kSingleSSConeAngle;
    private double kSingleSSCubeAngle = WristConstants.kSingleSSCubeAngle;
    
    private double kTransitoryAngle = WristConstants.kTransitoryAngle;

    public enum SmartMotionWristSpeed {REGULAR, SLOW, FAST};

    private double currentPIDSetpointAngle;

    private AbsoluteEncoder wristEncoder;

    public Wrist() {

        wristMotor = new CANSparkMax(RobotMap.kWristMotor, MotorType.kBrushless);
        wristMotor.setSmartCurrentLimit(WristConstants.kMaxCurrent);
        wristMotor.setIdleMode(IdleMode.kBrake);

        limitSensor = new DigitalInput(RobotMap.kWristLimitSensor);

        wristMotor.setInverted(true);

        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        wristMotor.setSoftLimit(SoftLimitDirection.kForward, WristConstants.kAngleMax);
        wristMotor.setSoftLimit(SoftLimitDirection.kReverse, WristConstants.kAngleMin);
        
        wristMotor.setClosedLoopRampRate(0.05); // use a 50 ms ramp rate on closed loop control

        // Keep track of the current setpoint for any position PID controllers (regular or SmartMotion by proxy)
        currentPIDSetpointAngle = WristConstants.kHomeAngle;

        wristEncoder = wristMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        wristEncoder.setPositionConversionFactor(WristConstants.kAbsoluteEncoderConversionFactor);
        wristEncoder.setInverted(true);
        wristEncoder.setZeroOffset(WristConstants.kAbsoluteEncoderZeroOffset);
        wristEncoder.setAverageDepth(32);

        pidController = wristMotor.getPIDController();
        pidController.setFeedbackDevice(wristEncoder);

        // Set up SmartMotion PIDController (regular speed) on pid slot 0
        kP = WristConstants.kP;
        kI = WristConstants.kI;
        kD = WristConstants.kD;
        kIz = WristConstants.kIz;
        kFF = WristConstants.kFF;
        setupPIDController(kP, kI, kD, kIz, kFF, 0);

        kSmartMotionRegularSetpointTol = WristConstants.kSmartMotionRegularSetpointTol;
        kSmartMotionRegularMinVel = WristConstants.kSmartMotionRegularMinVel;
        kSmartMotionRegularMaxVel = WristConstants.kSmartMotionRegularMaxVel;
        kSmartMotionRegularMaxAccel = WristConstants.kSmartMotionRegularMaxAccel;
        setRegularSmartMotionParameters(WristConstants.kSmartMotionRegularSetpointTol,
        WristConstants.kSmartMotionRegularMinVel, WristConstants.kSmartMotionRegularMaxVel, WristConstants.kSmartMotionRegularMaxAccel);

        // Set up SmartMotion PIDController (slow speed) on pid slot 2
        setupPIDController(kP, kI, kD, kIz, kFF, 2);
        kSmartMotionSlowSetpointTol = WristConstants.kSmartMotionSlowSetpointTol;
        kSmartMotionSlowMinVel = WristConstants.kSmartMotionSlowMinVel;
        kSmartMotionSlowMaxVel = WristConstants.kSmartMotionSlowMaxVel;
        kSmartMotionSlowMaxAccel = WristConstants.kSmartMotionSlowMaxAccel;
        setSlowSmartMotionParameters(WristConstants.kSmartMotionSlowSetpointTol,
        WristConstants.kSmartMotionSlowMinVel, WristConstants.kSmartMotionSlowMaxVel, WristConstants.kSmartMotionSlowMaxAccel);

        // Set up SmartMotion PIDController (fast speed) on pid slot 3
        setupPIDController(kP, kI, kD, kIz, kFF, 3);
        kSmartMotionFastSetpointTol = WristConstants.kSmartMotionFastSetpointTol;
        kSmartMotionFastMinVel = WristConstants.kSmartMotionFastMinVel;
        kSmartMotionFastMaxVel = WristConstants.kSmartMotionFastMaxVel;
        kSmartMotionFastMaxAccel = WristConstants.kSmartMotionFastMaxAccel;
        setFastSmartMotionParameters(WristConstants.kSmartMotionFastSetpointTol,
        WristConstants.kSmartMotionFastMinVel, WristConstants.kSmartMotionFastMaxVel, WristConstants.kSmartMotionFastMaxAccel);

        // Set up position PIDController on pid slot 1
        kPositionP = WristConstants.kPositionP;
        kPositionI = WristConstants.kPositionI;
        kPositionD = WristConstants.kPositionD;
        kPositionIz = WristConstants.kPositionIz;
        setupPIDController(kPositionP, kPositionI, kPositionD, kPositionIz, 1);

        kG = WristConstants.kGVolts;
        kV = WristConstants.kVVoltSecondPerRad;
        kA = WristConstants.kAVoltSecondSquaredPerRad;
        wristFeedforward = new ArmFeedforward(0.0, kG, kV, kA);

        wristMotor.getEncoder().setPositionConversionFactor(WristConstants.kEncoderConversionFactor);
        wristMotor.getEncoder().setVelocityConversionFactor(1.0);
        setEncoder(WristConstants.kHomeAngle);

        wristMotor.burnFlash();

        currentAverage = new RollingAverage(10);
        monitorCurrent = false;

    }

    public void setRegularSmartMotionParameters(double setpointTol, double minVel, double maxVel, double maxAccel){
        kSmartMotionRegularSetpointTol = setpointTol;
        kSmartMotionRegularMinVel = minVel;
        kSmartMotionRegularMaxVel = maxVel;
        kSmartMotionRegularMaxAccel = maxAccel;
        pidController.setSmartMotionAllowedClosedLoopError(kSmartMotionRegularSetpointTol, 0);
        pidController.setSmartMotionMinOutputVelocity(kSmartMotionRegularMinVel, 0);
        pidController.setSmartMotionMaxVelocity(kSmartMotionRegularMaxVel, 0);
        pidController.setSmartMotionMaxAccel(kSmartMotionRegularMaxAccel, 0);
    }

    public void setSlowSmartMotionParameters(double setpointTol, double minVel, double maxVel, double maxAccel){
        kSmartMotionSlowSetpointTol = setpointTol;
        kSmartMotionSlowMinVel = minVel;
        kSmartMotionSlowMaxVel = maxVel;
        kSmartMotionSlowMaxAccel = maxAccel;
        pidController.setSmartMotionAllowedClosedLoopError(kSmartMotionSlowSetpointTol, 2);
        pidController.setSmartMotionMinOutputVelocity(kSmartMotionSlowMinVel, 2);
        pidController.setSmartMotionMaxVelocity(kSmartMotionSlowMaxVel, 2);
        pidController.setSmartMotionMaxAccel(kSmartMotionSlowMaxAccel, 2);
    }

    public void setFastSmartMotionParameters(double setpointTol, double minVel, double maxVel, double maxAccel){
        kSmartMotionFastSetpointTol = setpointTol;
        kSmartMotionFastMinVel = minVel;
        kSmartMotionFastMaxVel = maxVel;
        kSmartMotionFastMaxAccel = maxAccel;
        pidController.setSmartMotionAllowedClosedLoopError(kSmartMotionFastSetpointTol, 3);
        pidController.setSmartMotionMinOutputVelocity(kSmartMotionFastMinVel, 3);
        pidController.setSmartMotionMaxVelocity(kSmartMotionFastMaxVel, 3);
        pidController.setSmartMotionMaxAccel(kSmartMotionFastMaxAccel, 3);
    }

    public void setPosition(double setpointDeg) {
        currentPIDSetpointAngle = setpointDeg;

        // arbitraryFF = wristFeedforward.calculate(Math.toRadians(setpointDeg), 0);

        if(setpointDeg >= WristConstants.kAngleMin && setpointDeg <= WristConstants.kAngleMax){
            pidController.setReference(setpointDeg, ControlType.kPosition, 1, 0);
        }
    }

    // Velocity PID is currently only used for testing Smart Motion velocity tuning
    public void setVelocity(double setpointVel){
        arbitraryFF = wristFeedforward.calculate(Math.toRadians(108-wrist.getPosition()), 0);

        pidController.setReference(setpointVel, ControlType.kVelocity, 0, arbitraryFF);
    }

    public void setPositionSmartMotion(double setpointDeg, SmartMotionWristSpeed mode){
        currentPIDSetpointAngle = setpointDeg;

        arbitraryFF = wristFeedforward.calculate(Math.toRadians(wrist.getPosition()), 0);

        switch(mode){
            case REGULAR:
                pidController.setReference(setpointDeg, ControlType.kSmartMotion, 0, arbitraryFF);
                break;
            case SLOW:
                pidController.setReference(setpointDeg, ControlType.kSmartMotion, 2, arbitraryFF);
                break;
            case FAST:
                pidController.setReference(setpointDeg, ControlType.kSmartMotion, 3, arbitraryFF);
                break;
            default:
                pidController.setReference(setpointDeg, ControlType.kSmartMotion, 0, arbitraryFF);
        }
    }

    public double getArbitraryFF() {
        return arbitraryFF;
    }

    public void updatePIDController(double p, double i, double d, double izone, int pidslot) {
        updatePIDController(p, i, d, izone, kFF, pidslot);
    }

    public void updatePIDController(double p, double i, double d, double izone, double ff, int pidslot) {
        if(pidslot == 0){
            if(kP != p || kI != i || kD != d || kIz != izone || kFF != ff){
                kP = p;
                kI = i;
                kD = d;
                kIz = izone;
                kFF = ff;
                pidController.setP(p, pidslot);
                pidController.setI(i, pidslot);
                pidController.setD(d, pidslot);
                pidController.setIZone(izone, pidslot);
                pidController.setFF(ff, pidslot);

            }
        }
        else if(pidslot == 1){
            if(kPositionP != p || kPositionI != i || kPositionD != d || kPositionIz != izone){
                kPositionP = p;
                kPositionI = i;
                kPositionD = d;
                kPositionIz = izone;

                pidController.setP(p, pidslot);
                pidController.setI(i, pidslot);
                pidController.setD(d, pidslot);
                pidController.setIZone(izone, pidslot);

            }           
        }
        else if(pidslot == 2){
            if(kP != p || kI != i || kD != d || kIz != izone || kFF != ff){
                kP = p;
                kI = i;
                kD = d;
                kIz = izone;
                kFF = ff;

                pidController.setP(p, pidslot);
                pidController.setI(i, pidslot);
                pidController.setD(d, pidslot);
                pidController.setIZone(izone, pidslot);
                pidController.setFF(ff, pidslot);
            }
        }
        else if(pidslot == 3){
            if(kP != p || kI != i || kD != d || kIz != izone || kFF != ff){
                kP = p;
                kI = i;
                kD = d;
                kIz = izone;
                kFF = ff;

                pidController.setP(p, pidslot);
                pidController.setI(i, pidslot);
                pidController.setD(d, pidslot);
                pidController.setIZone(izone, pidslot);
                pidController.setFF(ff, pidslot);

            }
        }
    }

    public boolean atLimitSensor(){
    return !limitSensor.get();
    }

    public void resetEncoder() {
        setEncoder(0);
    }

    public void setEncoder(double newEncoderValue) {
        wristMotor.getEncoder().setPosition(newEncoderValue);
    }

    public void setPercentOutput(double speed) {
        wristMotor.set(speed);
    }

    public double getMotorTemperature() {
        return wristMotor.getMotorTemperature();
    }

    public double getSpeed() {
        return wristMotor.get();
    }

    public double getOutputCurrent() {
        return wristMotor.getOutputCurrent();
    }

    public double getPosition() {
        // return wristMotor.getEncoder().getPosition();
        return wristEncoder.getPosition(); // + 96.5; //- WristConstants.kWristAbsoluteEncoderAngleOffset;
    }

    public double getVelocity() {
        return wristMotor.getEncoder().getVelocity();
    }

    public void stopWrist() {
        wristMotor.set(0);
    }

    public void updateWristFeedforward(double dbkg, double dbkv, double dbka) {
        if (kG != dbkg || kV != dbkv || kA != dbka) {
            kG = dbkg;
            kV = dbkv;
            kA = dbka;
            wristFeedforward = new ArmFeedforward(0, kG, kV, kA);
        }
    }

    public void setupPIDController(double p, double i, double d, double izone, int pidslot) {
        pidController.setP(p, pidslot);
        pidController.setI(i, pidslot);
        pidController.setD(d, pidslot);
        pidController.setIZone(izone, pidslot);
    }

    public void setupPIDController(double p, double i, double d, double izone, double ff, int pidslot) {
        pidController.setP(p, pidslot);
        pidController.setI(i, pidslot);
        pidController.setD(d, pidslot);
        pidController.setIZone(izone, pidslot);
        pidController.setFF(ff, pidslot);
    }

    public void periodic() {
        if(monitorCurrent){
            currentAverage.add(wristMotor.getOutputCurrent());
        }
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

    public void setMode(IdleMode mode) {
        wristMotor.setIdleMode(mode);
    }

    public static Wrist getInstance() {
        if (wrist == null) {
            wrist = new Wrist();
        }
        return wrist;
    }

    public double getVoltage(){
        return wristMotor.getAppliedOutput()*100;
    }

    public void disablePIDController() {
        wristMotor.set(0);
    }

    public double getkHomeAngle() {
        return kHomeAngle;
    }

    public void setkHomeAngle(double kHomeAngle) {
        this.kHomeAngle = kHomeAngle;
    }

    public double getkStowedAngle() {
        return kStowedAngle;
    }

    public void setkStowedAngle(double kStowedAngle) {
        this.kStowedAngle = kStowedAngle;
    }

    public double getkPreScoreAngle() {
        return kPreScoreAngle;
    }

    public void setkPreScoreAngle(double kPreScoreAngle) {
        this.kPreScoreAngle = kPreScoreAngle;
    }

    public double getkL1Angle() {
        return kL1Angle;
    }

    public void setkL1Angle(double kL1Angle) {
        this.kL1Angle = kL1Angle;
    }

    public double getkCompactFloorConeAngle() {
        return kCompactFloorConeAngle;
    }

    public void setkCompactFloorConeAngle(double kCompactFloorConeAngle) {
        this.kCompactFloorConeAngle = kCompactFloorConeAngle;
    }

    public double getkCompactFloorCubeAngle() {
        return kCompactFloorCubeAngle;
    }

    public void setkCompactFloorCubeAngle(double kCompactFloorCubeAngle) {
        this.kCompactFloorCubeAngle = kCompactFloorCubeAngle;
    }

    public double getkExtendedFloorConeAngle() {
        return kExtendedFloorConeAngle;
    }

    public void setkExtendedFloorConeAngle(double kExtendedFloorConeAngle) {
        this.kExtendedFloorConeAngle = kExtendedFloorConeAngle;
    }

    public double getkExtendedFloorCubeAngle() {
        return kExtendedFloorCubeAngle;
    }

    public void setkExtendedFloorCubeAngle(double kExtendedFloorCubeAngle) {
        this.kExtendedFloorCubeAngle = kExtendedFloorCubeAngle;
    }

    public double getkL2ConeAngle() {
        return kL2ConeAngle;
    }

    public void setkL2ConeAngle(double kL2ConeAngle) {
        this.kL2ConeAngle = kL2ConeAngle;
    }

    public double getkL2CubeAngle() {
        return kL2CubeAngle;
    }

    public void setkL2CubeAngle(double kL2CubeAngle) {
        this.kL2CubeAngle = kL2CubeAngle;
    }

    public double getkL3CubeForwardAngle() {
        return kL3CubeForwardAngle;
    }

    public void setkL3CubeForwardAngle(double kL3CubeForwardAngle) {
        this.kL3CubeForwardAngle = kL3CubeForwardAngle;
    }

    public double getkL3CubeInvertedAngle() {
        return kL3CubeInvertedAngle;
    }

    public void setkL3CubeInvertedAngle(double kL3CubeInvertedAngle) {
        this.kL3CubeInvertedAngle = kL3CubeInvertedAngle;
    }

    public double getkL3ConeForwardAngle() {
        return kL3ConeForwardAngle;
    }

    public void setkL3ConeForwardAngle(double kL3ConeForwardAngle) {
        this.kL3ConeForwardAngle = kL3ConeForwardAngle;
    }

    public double getkL3ConeInvertedAngle() {
        return kL3ConeInvertedAngle;
    }

    public void setkL3ConeInvertedAngle(double kL3ConeInvertedAngle) {
        this.kL3ConeInvertedAngle = kL3ConeInvertedAngle;
    }

    public double getkDoubleSSConeAngle() {
        return kDoubleSSConeAngle;
    }

    public void setkDoubleSSConeAngle(double kDoubleSSConeAngle) {
        this.kDoubleSSConeAngle = kDoubleSSConeAngle;
    }

    public double getkSingleSSConeAngle() {
        return kSingleSSConeAngle;
    }

    public void setkSingleSSConeAngle(double kSingleSSConeAngle) {
        this.kSingleSSConeAngle = kSingleSSConeAngle;
    }

    public double getkSingleSSCubeAngle() {
        return kSingleSSCubeAngle;
    }

    public void setkSingleSSCubeAngle(double kSingleSSCubeAngle) {
        this.kSingleSSCubeAngle = kSingleSSCubeAngle;
    }

    public double getkTransitoryAngle() {
        return kTransitoryAngle;
    }

    public void setkTransitoryAngle(double kTransitoryAngle) {
        this.kTransitoryAngle = kTransitoryAngle;
    }

    public void turnOnSmartLimits(){
        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public void turnOffSmartLimits(){
        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

}