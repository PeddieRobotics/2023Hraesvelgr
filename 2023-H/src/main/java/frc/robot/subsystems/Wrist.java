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

    // Angles (poses) start here
    private double kHomeAngle = WristConstants.kHomeAngle;
    private double kStowedAngle = WristConstants.kStowedAngle;
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
    private double kL3ConeAngle = WristConstants.kL3ConeAngle;

    private double kLLSeekAngle = WristConstants.kLLSeekAngle;
    private double kDoubleSSConeAngle = WristConstants.kDoubleSSConeAngle;
    private double kSingleSSAngle = WristConstants.kSingleSSAngle;
    private double kTransitoryAngle = WristConstants.kTransitoryAngle;


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

    public double getkL3ConeAngle() {
        return kL3ConeAngle;
    }

    public void setkL3ConeAngle(double kL3ConeAngle) {
        this.kL3ConeAngle = kL3ConeAngle;
    }

    public double getkLLSeekAngle() {
        return kLLSeekAngle;
    }

    public void setkLLSeekAngle(double kLLSeekAngle) {
        this.kLLSeekAngle = kLLSeekAngle;
    }

    public double getkDoubleSSConeAngle() {
        return kDoubleSSConeAngle;
    }

    public void setkDoubleSSConeAngle(double kDoubleSSConeAngle) {
        this.kDoubleSSConeAngle = kDoubleSSConeAngle;
    }

    public double getkSingleSSAngle() {
        return kSingleSSAngle;
    }

    public void setkSingleSSAngle(double kSingleSSAngle) {
        this.kSingleSSAngle = kSingleSSAngle;
    }

    public double getkTransitoryAngle() {
        return kTransitoryAngle;
    }

    public void setkTransitoryAngle(double kTransitoryAngle) {
        this.kTransitoryAngle = kTransitoryAngle;
    }

}