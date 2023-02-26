package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.WristConstants;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Wrist {
    private static Wrist wrist;

    private CANSparkMax wristMotor;

    // private DigitalInput limitSensor;

    private SparkMaxPIDController pidController;

    private ArmFeedforward wristFeedforward;

    private double kP, kI, kD, kIz, kG, kV, kA, arbitraryFF;

    public Wrist() {

        wristMotor = new CANSparkMax(RobotMap.kWristMotor, MotorType.kBrushless);
        wristMotor.setSmartCurrentLimit(WristConstants.kMaxCurrent);
        wristMotor.setIdleMode(IdleMode.kBrake);

        pidController = wristMotor.getPIDController();

        kP = WristConstants.kP;
        kI = WristConstants.kI;
        kD = WristConstants.kD;
        kIz = WristConstants.kIz;
        setupPIDController(kP, kI, kD, kIz, 0);

        kG = WristConstants.kGVolts;
        kV = WristConstants.kVVoltSecondPerRad;
        kA = WristConstants.kAVoltSecondSquaredPerRad;

        wristFeedforward = new ArmFeedforward(0.0, kG, kV, kA);

        wristMotor.getEncoder().setPositionConversionFactor(WristConstants.kEncoderConversionFactor);
        wristMotor.getEncoder().setVelocityConversionFactor(WristConstants.kEncoderConversionFactor/60.0);
        setEncoder(WristConstants.kHomeAngle);

        // limitSensor = new DigitalInput(RobotMap.kWristLimitSensor);

        wristMotor.setSoftLimit(SoftLimitDirection.kForward, WristConstants.kAngleMax);
        wristMotor.setSoftLimit(SoftLimitDirection.kReverse, WristConstants.kAngleMin);

        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        //
        wristMotor.setClosedLoopRampRate(0.01);

    }

    public void setPosition(double setpointDeg) {
        arbitraryFF = wristFeedforward.calculate(Math.toRadians(setpointDeg), 0);

        pidController.setReference(setpointDeg, ControlType.kPosition, 0, arbitraryFF);
    }

    public double getArbitraryFF() {
        return arbitraryFF;
    }

    // public boolean atLimitSensor(){
    // return !limitSensor.get();
    // }

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
        return wristMotor.getEncoder().getPosition();
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

    public void updatePIDController(double p, double i, double d, double izone, int pidslot) {
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

    public void periodic() {

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
}