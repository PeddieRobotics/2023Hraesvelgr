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
    kPositionIz, kG, kV, kA, arbitraryFF, kSmartMotionSetpointTol, kSmartMotionMinVel,
    kSmartMotionMaxVel, kSmartMotionMaxAccel;

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
  
          // Set up SmartMotion PIDController on pid slot 0
          kP = ShoulderConstants.kP;
          kI = ShoulderConstants.kI;
          kD = ShoulderConstants.kD;
          kIz = ShoulderConstants.kIz;
          setPIDController(kP, kI, kD, kIz, 0);
  
          kSmartMotionSetpointTol = ShoulderConstants.kSmartMotionSetpointTol;
          kSmartMotionMinVel = ShoulderConstants.kSmartMotionMinVel;
          kSmartMotionMaxVel = ShoulderConstants.kSmartMotionMaxVel;
          kSmartMotionMaxAccel = ShoulderConstants.kSmartMotionMaxAccel;
          setSmartMotionParameters(ShoulderConstants.kSmartMotionSetpointTol,
          ShoulderConstants.kSmartMotionMinVel, ShoulderConstants.kSmartMotionMaxVel, ShoulderConstants.kSmartMotionMaxAccel);
  
          // Set up position PIDController on pid slot 1
          kPositionP = ShoulderConstants.kPositionP;
          kPositionI = ShoulderConstants.kPositionI;
          kPositionD = ShoulderConstants.kPositionD;
          kPositionIz = ShoulderConstants.kPositionIz;
          setPIDController(kPositionP, kPositionI, kPositionD, kPositionIz, 1);
  
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

    public void setSmartMotionParameters(double setpointTol, double minVel, double maxVel, double maxAccel){
        kSmartMotionSetpointTol = setpointTol;
        kSmartMotionMinVel = minVel;
        kSmartMotionMaxVel = maxVel;
        kSmartMotionMaxAccel = maxAccel;
        pidController.setSmartMotionAllowedClosedLoopError(kSmartMotionSetpointTol, 0);
        pidController.setSmartMotionMinOutputVelocity(kSmartMotionMinVel, 0);
        pidController.setSmartMotionMaxVelocity(kSmartMotionMaxVel, 0);
        pidController.setSmartMotionMaxAccel(kSmartMotionMaxAccel, 0);

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
        // Below the fulcrum, disable feedforward
        if(getAngle() < -50.0){
             arbitraryFF = 0.0;
             pidController.setReference(setpointDeg, ControlType.kPosition, 1, arbitraryFF);
        }   
        else{
            arbitraryFF = shoulderFeedforward.calculate(Math.toRadians(shoulder.getAngle()), 0);

            pidController.setReference(setpointDeg, ControlType.kPosition, 1, arbitraryFF);
        }
    }

    public void setVelocity(double setpointVel){
        arbitraryFF = shoulderFeedforward.calculate(Math.toRadians(shoulder.getAngle()), Math.toRadians(setpointVel));

        pidController.setReference(setpointVel, ControlType.kVelocity, 0, arbitraryFF);
    }

    public void setPositionSmartMotion(double setpointDeg){
        arbitraryFF = shoulderFeedforward.calculate(Math.toRadians(shoulder.getAngle()), 0);

        pidController.setReference(setpointDeg, ControlType.kSmartMotion, 0, arbitraryFF);
    }

    public double getArbitraryFF() {
        return arbitraryFF;
    }

    public boolean atLimitSensor(){
    return !limitSensor.get();
    }

    public void resetEncoder() {
        setEncoder(ShoulderConstants.kHomeAngle+2);
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

    // Not really recommended(?) but will ensure all PID controllers are disabled.
    // Currently unused.
    public void disablePIDController() {
        setShoulderFeedforward(0, 0, 0);
        setPIDController(0, 0, 0, 0, 0); // Smart motion PID controller
        setPIDController(0, 0, 0, 0, 1); // Position PID controller
    }

    public void setShoulderFeedforward(double dbkg, double dbkv, double dbka) {
        if (kG != dbkg || kV != dbkv || kA != dbka) {
            kG = dbkg;
            kV = dbkv;
            kA = dbka;
            shoulderFeedforward = new ArmFeedforward(0, kG, kV, kA);
        }
    }

    public void setPIDController(double p, double i, double d, double izone, int pidslot) {
        pidController.setP(p, pidslot);
        pidController.setI(i, pidslot);
        pidController.setD(d, pidslot);
        pidController.setIZone(izone, pidslot);
    }

    public void periodic() {

        // if(atLimitSensor()){
        //     reachedLimitSensor = true;
        // }
        // else if(reachedLimitSensor && !atLimitSensor()){
        //     shoulder.resetEncoder();
        //     reachedLimitSensor = false;
        // }
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