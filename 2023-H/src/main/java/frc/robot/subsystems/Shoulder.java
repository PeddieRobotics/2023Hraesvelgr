package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.RobotMap;
import frc.robot.utils.RollingAverage;

public class Shoulder {

    private static Shoulder shoulder;

    private CANSparkMax shoulderMotorMaster, shoulderMotorFollower;

    private SparkMaxPIDController pidController;
    private ArmFeedforward shoulderFeedforward;

    private DigitalInput limitSensor;

    private RollingAverage currentAverage;
    private boolean monitorCurrent;

    private double kP, kI, kD, kIz, kFF, kPositionP, kPositionI, kPositionD,
    kPositionIz, kG, kV, kA, arbitraryFF, kSmartMotionRegularSetpointTol, kSmartMotionRegularMinVel,
    kSmartMotionRegularMaxVel, kSmartMotionRegularMaxAccel, kSmartMotionSlowSetpointTol, kSmartMotionSlowMinVel,
    kSmartMotionSlowMaxVel, kSmartMotionSlowMaxAccel, kSmartMotionFastSetpointTol, kSmartMotionFastMinVel,
    kSmartMotionFastMaxVel, kSmartMotionFastMaxAccel;

    // Angles (poses) start here
    private double kHomeAngle = ShoulderConstants.kHomeAngle;
    private double kStowedAngle = ShoulderConstants.kStowedAngle;
    private double kPreScoreAngle = ShoulderConstants.kPreScoreAngle;

    private double kL1Angle = ShoulderConstants.kL1Angle;

    // Shoulder is not fully extended out
    private double kCompactFloorConeAngle = ShoulderConstants.kCompactFloorConeAngle;
    private double kCompactFloorCubeAngle = ShoulderConstants.kCompactFloorCubeAngle;

    // Shoulder is fully extended out
    private double kExtendedFloorConeAngle = ShoulderConstants.kExtendedFloorConeAngle;
    private double kExtendedFloorCubeAngle = ShoulderConstants.kExtendedFloorCubeAngle;

    private double kL2ConeAngle = ShoulderConstants.kL2ConeAngle;
    private double kL2CubeAngle = ShoulderConstants.kL2CubeAngle;

    private double kL3CubeForwardAngle = ShoulderConstants.kL3CubeForwardAngle;
    private double kL3CubeInvertedAngle = ShoulderConstants.kL3CubeInvertedAngle;
    private double kL3ConeForwardAngle = ShoulderConstants.kL3ConeForwardAngle;

    private double kL3ConeInvertedAngle = ShoulderConstants.kL3ConeInvertedAngle;

    private double kDoubleSSConeAngle = ShoulderConstants.kDoubleSSConeAngle;
    private double kSingleSSConeAngle = ShoulderConstants.kSingleSSConeAngle;
    private double kSingleSSCubeAngle = ShoulderConstants.kSingleSSCubeAngle;

    private double kTransitoryAngle = ShoulderConstants.kTransitoryAngle;

    public enum SmartMotionArmSpeed {REGULAR, SLOW, FAST};

    private double currentPIDSetpointAngle;

    public Shoulder() {

          // Basic motor configuration
          shoulderMotorMaster = new CANSparkMax(RobotMap.kShoulderMotorMaster, MotorType.kBrushless);
          shoulderMotorMaster.setSmartCurrentLimit(ShoulderConstants.kMaxCurrent);
  
          shoulderMotorFollower = new CANSparkMax(RobotMap.kShoulderMotorFollower, MotorType.kBrushless);
          shoulderMotorFollower.setSmartCurrentLimit(ShoulderConstants.kMaxCurrent);
  
          shoulderMotorFollower.follow(shoulderMotorMaster);
  
          shoulderMotorMaster.setIdleMode(IdleMode.kCoast);
          shoulderMotorFollower.setIdleMode(IdleMode.kCoast);
  
          shoulderMotorMaster.setInverted(true);
          shoulderMotorFollower.setInverted(true);
  
          shoulderMotorMaster.getEncoder().setPositionConversionFactor(ShoulderConstants.kEncoderConversionFactor);
          shoulderMotorFollower.getEncoder().setPositionConversionFactor(ShoulderConstants.kEncoderConversionFactor);
          setEncoder(ShoulderConstants.kHomeAngle);
  
          shoulderMotorMaster.getEncoder().setVelocityConversionFactor(1.0);
          shoulderMotorFollower.getEncoder().setVelocityConversionFactor(1.0);
  
          // Safety: ramp rate and soft limits
          shoulderMotorMaster.setClosedLoopRampRate(0.05); // use a 50 ms ramp rate on closed loop control
  
          shoulderMotorMaster.setSoftLimit(SoftLimitDirection.kForward, 155);
          shoulderMotorMaster.setSoftLimit(SoftLimitDirection.kReverse, -75);
  
          shoulderMotorFollower.setSoftLimit(SoftLimitDirection.kForward, 155);
          shoulderMotorFollower.setSoftLimit(SoftLimitDirection.kReverse, -75);
  
          shoulderMotorMaster.enableSoftLimit(SoftLimitDirection.kForward, true);
          shoulderMotorMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);
          shoulderMotorFollower.enableSoftLimit(SoftLimitDirection.kForward, true);
          shoulderMotorFollower.enableSoftLimit(SoftLimitDirection.kReverse, true);
  
          // PID control setup
          pidController = shoulderMotorMaster.getPIDController();
  
          // Set up SmartMotion PIDController (regular speed) on pid slot 0
          kP = ShoulderConstants.kP;
          kI = ShoulderConstants.kI;
          kD = ShoulderConstants.kD;
          kIz = ShoulderConstants.kIz;
          kFF = ShoulderConstants.kFF;
          setupPIDController(kP, kI, kD, kIz, kFF, 0);

          kSmartMotionRegularSetpointTol = ShoulderConstants.kSmartMotionRegularSetpointTol;
          kSmartMotionRegularMinVel = ShoulderConstants.kSmartMotionRegularMinVel;
          kSmartMotionRegularMaxVel = ShoulderConstants.kSmartMotionRegularMaxVel;
          kSmartMotionRegularMaxAccel = ShoulderConstants.kSmartMotionRegularMaxAccel;
          setRegularSmartMotionParameters(ShoulderConstants.kSmartMotionRegularSetpointTol,
          ShoulderConstants.kSmartMotionRegularMinVel, ShoulderConstants.kSmartMotionRegularMaxVel, ShoulderConstants.kSmartMotionRegularMaxAccel);
  
          // Set up SmartMotion PIDController (slow speed) on pid slot 2
          setupPIDController(kP, kI, kD, kIz, kFF, 2);
          kSmartMotionSlowSetpointTol = ShoulderConstants.kSmartMotionSlowSetpointTol;
          kSmartMotionSlowMinVel = ShoulderConstants.kSmartMotionSlowMinVel;
          kSmartMotionSlowMaxVel = ShoulderConstants.kSmartMotionSlowMaxVel;
          kSmartMotionSlowMaxAccel = ShoulderConstants.kSmartMotionSlowMaxAccel;
          setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
          ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kSmartMotionSlowMaxVel, ShoulderConstants.kSmartMotionSlowMaxAccel);
  
          // Set up SmartMotion PIDController (fast speed) on pid slot 3
          setupPIDController(kP, kI, kD, kIz, kFF, 3);
          kSmartMotionFastSetpointTol = ShoulderConstants.kSmartMotionFastSetpointTol;
          kSmartMotionFastMinVel = ShoulderConstants.kSmartMotionFastMinVel;
          kSmartMotionFastMaxVel = ShoulderConstants.kSmartMotionFastMaxVel;
          kSmartMotionFastMaxAccel = ShoulderConstants.kSmartMotionFastMaxAccel;
          setFastSmartMotionParameters(ShoulderConstants.kSmartMotionFastSetpointTol,
          ShoulderConstants.kSmartMotionFastMinVel, ShoulderConstants.kSmartMotionFastMaxVel, ShoulderConstants.kSmartMotionFastMaxAccel);
  
          // Set up position PIDController on pid slot 1
          kPositionP = ShoulderConstants.kPositionP;
          kPositionI = ShoulderConstants.kPositionI;
          kPositionD = ShoulderConstants.kPositionD;
          kPositionIz = ShoulderConstants.kPositionIz;
          setupPIDController(kPositionP, kPositionI, kPositionD, kPositionIz, 1);
  
          // Set up shoulder feedforward params
          kG = ShoulderConstants.kGVolts;
          kV = ShoulderConstants.kVVoltSecondPerRad;
          kA = ShoulderConstants.kAVoltSecondSquaredPerRad;
          shoulderFeedforward = new ArmFeedforward(0.0, ShoulderConstants.kGVolts,
                  ShoulderConstants.kVVoltSecondPerRad, ShoulderConstants.kAVoltSecondSquaredPerRad);

        // Hall effect sensor for homing the shoulder
        limitSensor = new DigitalInput(RobotMap.kShoulderLimitSensor);

        // Keep track of the current setpoint for any position PID controllers (regular or SmartMotion by proxy)
        currentPIDSetpointAngle = ShoulderConstants.kHomeAngle;

        shoulderMotorMaster.burnFlash();
        shoulderMotorFollower.burnFlash();

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

    public double getAngle() {
        return shoulderMotorMaster.getEncoder().getPosition();
    }

    public double getOutputCurrent() {
        return shoulderMotorMaster.getOutputCurrent();
    }

    public void stopShoulder() {
        shoulderMotorMaster.set(0);
    }

    // Currently only used to hold the position of the arm after a smart motion call.
    // See Arm class method holdPosition
    public void setPosition(double setpointDeg) {
        currentPIDSetpointAngle = setpointDeg;

        arbitraryFF = shoulderFeedforward.calculate(Math.toRadians(shoulder.getAngle()), 0);

        pidController.setReference(setpointDeg, ControlType.kPosition, 1, arbitraryFF);
    }

    // Velocity PID is currently only used for testing Smart Motion velocity tuning
    public void setVelocity(double setpointVel){
        arbitraryFF = shoulderFeedforward.calculate(Math.toRadians(shoulder.getAngle()), 0);

        pidController.setReference(setpointVel, ControlType.kVelocity, 0, arbitraryFF);
    }

    public void setPositionSmartMotion(double setpointDeg, SmartMotionArmSpeed mode){
        currentPIDSetpointAngle = setpointDeg;

        arbitraryFF = shoulderFeedforward.calculate(Math.toRadians(shoulder.getAngle()), 0);

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

    public boolean atLimitSensor(){
    return !limitSensor.get();
    }

    public void setEncoder(double newEncoderValue) {
        shoulderMotorMaster.getEncoder().setPosition(newEncoderValue);
        shoulderMotorFollower.getEncoder().setPosition(newEncoderValue);
    }

    public double getPercentOutput() {
        return shoulderMotorMaster.get();
    }

    public void setPercentOutput(double speed) {
        shoulderMotorMaster.set(speed);
    }

    public double getMotorTemperature() {
        return shoulderMotorMaster.getMotorTemperature();
    }

    public double getPosition() {
        return shoulderMotorMaster.getEncoder().getPosition();
    }

    public double getVelocity() {
        return shoulderMotorMaster.getEncoder().getVelocity();
    }

    public void disablePIDController() {
        shoulderMotorMaster.set(0);
    }

    public void updateShoulderFeedforward(double dbkg, double dbkv, double dbka) {
        if (kG != dbkg || kV != dbkv || kA != dbka) {
            kG = dbkg;
            kV = dbkv;
            kA = dbka;
            shoulderFeedforward = new ArmFeedforward(0, kG, kV, kA);
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

    public void periodic() {
        if(monitorCurrent){
            currentAverage.add(shoulderMotorMaster.getOutputCurrent());
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
        shoulderMotorMaster.setIdleMode(mode);
        shoulderMotorFollower.setIdleMode(mode);
    }

    public static Shoulder getInstance() {
        if (shoulder == null) {
            shoulder = new Shoulder();
        }
        return shoulder;
    }

    public double getVoltage(){
        return shoulderMotorMaster.getAppliedOutput()*100;
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

    public double getCurrentPIDSetpointAngle() {
        return currentPIDSetpointAngle;
    }

    public void setCurrentPIDSetpointAngle(double currentPIDSetpointAngle) {
        this.currentPIDSetpointAngle = currentPIDSetpointAngle;
    }

    public void turnOnSmartLimits(){
        shoulderMotorMaster.enableSoftLimit(SoftLimitDirection.kForward, true);
        shoulderMotorMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);
        shoulderMotorFollower.enableSoftLimit(SoftLimitDirection.kForward, true);
        shoulderMotorFollower.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public void turnOffSmartLimits(){
        shoulderMotorMaster.enableSoftLimit(SoftLimitDirection.kForward, false);
        shoulderMotorMaster.enableSoftLimit(SoftLimitDirection.kReverse, false);
        shoulderMotorFollower.enableSoftLimit(SoftLimitDirection.kForward, false);
        shoulderMotorFollower.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }
}