package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMapH;
import frc.robot.utils.Constants.WristConstants;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Wrist {
    private static Wrist wrist;

    private CANSparkMax wristMotor;

    private DigitalInput limitSensor;

    private SparkMaxPIDController pidController;

    private ArmFeedforward armFeedforward;

    private double kS, kG, kV, kA, dynamicFeedforward;

    protected Wrist() {
        wristMotor = new CANSparkMax(RobotMapH.kWristMotor, MotorType.kBrushless);
        wristMotor.setSmartCurrentLimit(WristConstants.kMaxCurrent);

        wristMotor.setIdleMode(IdleMode.kBrake);

        pidController = wristMotor.getPIDController();

        kS = WristConstants.kSVolts;
        kG = WristConstants.kGVolts;

        kV = WristConstants.kVVoltSecondPerRad;
        kA = WristConstants.kAVoltSecondSquaredPerRad;

        armFeedforward = new ArmFeedforward(kS, kG, kV, kA);

        limitSensor = new DigitalInput(RobotMapH.kWristLimitSensor);
    }

    public void setPosition(double setpoint) {
        dynamicFeedforward = armFeedforward.calculate(Math.toRadians(90.0 - setpoint), 0);


        pidController.setReference(setpoint, ControlType.kPosition, 0, dynamicFeedforward);
    }

    public double getDynamicFeedForward(){
        return dynamicFeedforward;
    }

    public boolean atLimitSensor(){
        return !limitSensor.get();
    }

    public void setVelocity(double setpoint) {
        pidController.setReference(setpoint, ControlType.kVelocity);
    }

    public void resetEncoder() {
        wristMotor.getEncoder().setPosition(0);
        // SmartDashboard.putBoolean("is encoder reset", true);
    }

    public void setMotor(double speed) {
        if(speed > 0){
            wristMotor.set(speed);
        } else {
            if(!atLimitSensor()){
                wristMotor.set(speed);
            } else {
                wristMotor.set(0);
            }
        }
    }

    public double getMotorTemperature(){
        return wristMotor.getMotorTemperature();
    }

    public double getSpeed(){
        return wristMotor.get();
    }

    public double getOutputCurrent(){
        return wristMotor.getOutputCurrent();
    }

    public double getPosition(){
        return wristMotor.getEncoder().getPosition();
    }

    public double getVelocity(){
        return wristMotor.getEncoder().getVelocity();
    }

    public void stopWrist(){
        wristMotor.set(0);
    }

    public void setArmFeedForward(double dbks, double dbkg, double dbkv, double dbka, boolean pidActive){
        if (pidActive && (kS != dbks || kG != dbkg || kV != dbkv || kA != dbka)) {
            kS = dbks;
            kG = dbkg;
            kV = dbkv;
            kA = dbka;
            armFeedforward = new ArmFeedforward(kS, kG, kV, kA);
        }
    }

    public void setPidController(double p, double i, double d, double ff,boolean pidActive){
        if(pidActive){
            pidController.setP(p);
            pidController.setI(i);
            pidController.setD(d);
            pidController.setFF(ff);
        }
    }

    public boolean isMoving(){
        return wristMotor.get() != 0.0;
    }

    public void periodic() {
        setArmFeedForward(SmartDashboard.getNumber("wrist kS", Constants.WristConstants.kSVolts),
                SmartDashboard.getNumber("wrist kG", Constants.WristConstants.kGVolts),
                SmartDashboard.getNumber("wrist kV", Constants.WristConstants.kVVoltSecondPerRad),
                SmartDashboard.getNumber("wrist kA", Constants.WristConstants.kAVoltSecondSquaredPerRad),
                SmartDashboard.getBoolean("wrist toggle pid active", false));

        setPidController(SmartDashboard.getNumber("wrist P", Constants.WristConstants.kP),
                SmartDashboard.getNumber("wrist I", Constants.WristConstants.kI),
                SmartDashboard.getNumber("wrist D", Constants.WristConstants.kD),
                SmartDashboard.getNumber("wrist FF", Constants.WristConstants.kFF),
                SmartDashboard.getBoolean("wrist toggle pid active", false));

        //setMotor(SmartDashboard.getNumber("wrist speed % setpoint", 0.0));
    }

    public CANSparkMax getMotor() {
        return wristMotor;
    }

    public static Wrist getInstance() {
        if(wrist == null){
            wrist = new Wrist();
        }
        return wrist;
    }
}