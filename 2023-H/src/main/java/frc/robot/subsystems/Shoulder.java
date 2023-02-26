package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.RobotMap;

public class Shoulder {

    private static Shoulder shoulder;

    private CANSparkMax shoulderMotorMaster, shoulderMotorFollower;

    private SparkMaxPIDController pidController;
    private ArmFeedforward shoulderFeedforward;

    private DigitalInput limitSensor;
    private boolean reachedLimitSensor;

    private double kP, kI, kD, kIz, kPositionP, kPositionI, kPositionD,
    kPositionIz, kG, kV, kA, arbitraryFF, kSmartMotionRegularSetpointTol, kSmartMotionRegularMinVel,
    kSmartMotionRegularMaxVel, kSmartMotionRegularMaxAccel, kSmartMotionSlowSetpointTol, kSmartMotionSlowMinVel,
    kSmartMotionSlowMaxVel, kSmartMotionSlowMaxAccel, kSmartMotionFastSetpointTol, kSmartMotionFastMinVel,
    kSmartMotionFastMaxVel, kSmartMotionFastMaxAccel;

    public enum SmartMotionArmSpeed {REGULAR, SLOW, FAST};

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
  
        //   shoulderMotorMaster.getEncoder().setVelocityConversionFactor(1);
        //   shoulderMotorFollower.getEncoder().setVelocityConversionFactor(1);
          
          shoulderMotorMaster.getEncoder().setVelocityConversionFactor(ShoulderConstants.kEncoderConversionFactor/60.0);
          shoulderMotorFollower.getEncoder().setVelocityConversionFactor(ShoulderConstants.kEncoderConversionFactor/60.0);
  
          // Safety: ramp rate and soft limits
          shoulderMotorMaster.setClosedLoopRampRate(0.01);
  
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
          setupPIDController(kP, kI, kD, kIz, 0);
  
          kSmartMotionRegularSetpointTol = ShoulderConstants.kSmartMotionRegularSetpointTol;
          kSmartMotionRegularMinVel = ShoulderConstants.kSmartMotionRegularMinVel;
          kSmartMotionRegularMaxVel = ShoulderConstants.kSmartMotionRegularMaxVel;
          kSmartMotionRegularMaxAccel = ShoulderConstants.kSmartMotionRegularMaxAccel;
          setRegularSmartMotionParameters(ShoulderConstants.kSmartMotionRegularSetpointTol,
          ShoulderConstants.kSmartMotionRegularMinVel, ShoulderConstants.kSmartMotionRegularMaxVel, ShoulderConstants.kSmartMotionRegularMaxAccel);
  
          // Set up SmartMotion PIDController (slow speed) on pid slot 2
          setupPIDController(kP, kI, kD, kIz, 2);
          kSmartMotionSlowSetpointTol = ShoulderConstants.kSmartMotionSlowSetpointTol;
          kSmartMotionSlowMinVel = ShoulderConstants.kSmartMotionSlowMinVel;
          kSmartMotionSlowMaxVel = ShoulderConstants.kSmartMotionSlowMaxVel;
          kSmartMotionSlowMaxAccel = ShoulderConstants.kSmartMotionSlowMaxAccel;
          setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
          ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kSmartMotionSlowMaxVel, ShoulderConstants.kSmartMotionSlowMaxAccel);
  
          // Set up SmartMotion PIDController (fast speed) on pid slot 3
          setupPIDController(kP, kI, kD, kIz, 3);
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
        reachedLimitSensor = false;

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
        shoulderMotorFollower.set(0);
    }

    // Currently only used to hold the position of the arm after a smart motion call.
    // See Arm class method holdPosition
    public void setPosition(double setpointDeg) {
        arbitraryFF = shoulderFeedforward.calculate(Math.toRadians(shoulder.getAngle()), 0);

        pidController.setReference(setpointDeg, ControlType.kPosition, 1, arbitraryFF);
    }

    public void setVelocity(double setpointVel){
        arbitraryFF = shoulderFeedforward.calculate(Math.toRadians(shoulder.getAngle()), Math.toRadians(setpointVel));

        pidController.setReference(setpointVel, ControlType.kVelocity, 0, arbitraryFF);
    }

    public void setPositionSmartMotion(double setpointDeg, SmartMotionArmSpeed mode){
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

    public void updatePIDController(double p, double i, double d, double izone, int pidslot) {
        if(pidslot == 0){
            if(kP != p || kI != i || kD != d || kIz != izone){
                kP = p;
                kI = i;
                kD = d;
                kIz = izone;
                pidController.setP(p, pidslot);
                pidController.setI(i, pidslot);
                pidController.setD(d, pidslot);
                pidController.setIZone(izone, pidslot);
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
            if(kP != p || kI != i || kD != d || kIz != izone){
                kP = p;
                kI = i;
                kD = d;
                kIz = izone;
                pidController.setP(p, pidslot);
                pidController.setI(i, pidslot);
                pidController.setD(d, pidslot);
                pidController.setIZone(izone, pidslot);
            }
        }
        else if(pidslot == 3){
            if(kP != p || kI != i || kD != d || kIz != izone){
                kP = p;
                kI = i;
                kD = d;
                kIz = izone;
                pidController.setP(p, pidslot);
                pidController.setI(i, pidslot);
                pidController.setD(d, pidslot);
                pidController.setIZone(izone, pidslot);
            }
        }
    }

    public void periodic() {
        // Limit sensor triggered and arm is moving down

        if(atLimitSensor()){   
            reachedLimitSensor = true;
        }

        // If the arm is moving up and leaves the limit sensor, reset the encoder
        if(reachedLimitSensor && !atLimitSensor() && getVelocity() > 0){
            // shoulder.setEncoder(ShoulderConstants.kHomeAngle+2); 
            reachedLimitSensor = false;
        }

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
}