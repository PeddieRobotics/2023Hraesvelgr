package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.RobotMap;

public class Shoulder {

    private static Shoulder shoulder;

    private CANSparkMax shoulderMotorMaster, shoulderMotorFollower;

    private SparkMaxPIDController pidController;
    private ArmFeedforward shoulderFeedforward;

    private double kP, kI, kD, kIz, kG, kV, kA, arbitraryFF, kSmartMotionSetpointTol, kSmartMotionMinVel, kSmartMotionMaxVel, kSmartMotionMaxAccel;

    public Shoulder() {

        shoulderMotorMaster = new CANSparkMax(RobotMap.kShoulderMotorMaster, MotorType.kBrushless);
        shoulderMotorMaster.setSmartCurrentLimit(ShoulderConstants.kMaxCurrent);

        shoulderMotorFollower = new CANSparkMax(RobotMap.kShoulderMotorFollower, MotorType.kBrushless);
        shoulderMotorFollower.setSmartCurrentLimit(ShoulderConstants.kMaxCurrent);

        shoulderMotorFollower.follow(shoulderMotorMaster);

        shoulderMotorMaster.setIdleMode(IdleMode.kCoast);
        shoulderMotorFollower.setIdleMode(IdleMode.kCoast);

        pidController = shoulderMotorMaster.getPIDController();
        kP = ShoulderConstants.kP;
        kI = ShoulderConstants.kI;
        kD = ShoulderConstants.kD;
        kIz = ShoulderConstants.kIz;
        setPIDController(kP, kI, kD, kIz);

        kSmartMotionSetpointTol = ShoulderConstants.kSmartMotionSetpointTol;
        kSmartMotionMinVel = ShoulderConstants.kSmartMotionMinVel;
        kSmartMotionMaxVel = ShoulderConstants.kSmartMotionMaxVel;
        kSmartMotionMaxAccel = ShoulderConstants.kSmartMotionMaxAccel;
        pidController.setSmartMotionAllowedClosedLoopError(kSmartMotionSetpointTol, 0);
        pidController.setSmartMotionMinOutputVelocity(kSmartMotionMinVel, 0);
        pidController.setSmartMotionMaxVelocity(kSmartMotionMaxVel, 0);
        pidController.setSmartMotionMaxAccel(kSmartMotionMaxAccel, 0);

        kG = ShoulderConstants.kGVolts;
        kV = ShoulderConstants.kVVoltSecondPerRad;
        kA = ShoulderConstants.kAVoltSecondSquaredPerRad;
        shoulderFeedforward = new ArmFeedforward(0.0, ShoulderConstants.kGVolts,
                ShoulderConstants.kVVoltSecondPerRad, ShoulderConstants.kAVoltSecondSquaredPerRad);

        shoulderMotorMaster.setInverted(true);
        shoulderMotorFollower.setInverted(true);

        shoulderMotorMaster.getEncoder().setPositionConversionFactor(ShoulderConstants.kEncoderConversionFactor);
        shoulderMotorFollower.getEncoder().setPositionConversionFactor(ShoulderConstants.kEncoderConversionFactor);
        setEncoder(-47.0);

        shoulderMotorMaster.getEncoder().setVelocityConversionFactor(ShoulderConstants.kEncoderConversionFactor/60.0);
        shoulderMotorFollower.getEncoder().setVelocityConversionFactor(ShoulderConstants.kEncoderConversionFactor/60.0);

        shoulderMotorMaster.setClosedLoopRampRate(0.01);

        shoulderMotorMaster.setSoftLimit(SoftLimitDirection.kForward, 120);
        shoulderMotorMaster.setSoftLimit(SoftLimitDirection.kReverse, -70);

        shoulderMotorFollower.setSoftLimit(SoftLimitDirection.kForward, 120);
        shoulderMotorFollower.setSoftLimit(SoftLimitDirection.kReverse, -70);

        shoulderMotorMaster.enableSoftLimit(SoftLimitDirection.kForward, true);
        shoulderMotorMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);
        shoulderMotorFollower.enableSoftLimit(SoftLimitDirection.kForward, true);
        shoulderMotorFollower.enableSoftLimit(SoftLimitDirection.kReverse, true);

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

    // Do not use without resetting PID constants to appropriate ones for a position loop.
    public void setPosition(double setpointDeg) {
        arbitraryFF = shoulderFeedforward.calculate(Math.toRadians(setpointDeg), 0);

        pidController.setReference(setpointDeg, ControlType.kPosition, 0, arbitraryFF);
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

    public void resetEncoder() {
        setEncoder(0);
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
        setShoulderFeedforward(0, 0, 0);
        setPIDController(0, 0, 0, 0);
    }

    public void setShoulderFeedforward(double dbkg, double dbkv, double dbka) {
        if (kG != dbkg || kV != dbkv || kA != dbka) {
            kG = dbkg;
            kV = dbkv;
            kA = dbka;
            shoulderFeedforward = new ArmFeedforward(0, kG, kV, kA);
        }
    }

    public void setPIDController(double p, double i, double d, double izone) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setIZone(izone);
    }

    public void periodic() {
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
}