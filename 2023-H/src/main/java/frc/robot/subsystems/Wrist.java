package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;
import frc.robot.utils.OI;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.WristConstants;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Wrist {
    private static Wrist wrist;

    private CANSparkMax wristMotor;

    // private DigitalInput limitSensor;

    private SparkMaxPIDController pidController;

    private ArmFeedforward wristFeedforward;

    private double kS, kG, kV, kA, arbitraryFF;

    protected Wrist() {
        wristMotor = new CANSparkMax(RobotMap.kWristMotor, MotorType.kBrushless);
        wristMotor.setSmartCurrentLimit(WristConstants.kMaxCurrent);
        wristMotor.setIdleMode(IdleMode.kBrake);

        pidController = wristMotor.getPIDController();

        kS = WristConstants.kSVolts;
        kG = WristConstants.kGVolts;

        kV = WristConstants.kVVoltSecondPerRad;
        kA = WristConstants.kAVoltSecondSquaredPerRad;

        wristFeedforward = new ArmFeedforward(kS, kG, kV, kA);

        wristMotor.getEncoder().setPositionConversionFactor(WristConstants.kWristEncoderConversionFactor);
        setEncoder(103.77);

        // limitSensor = new DigitalInput(RobotMap.kWristLimitSensor);

        wristMotor.setSoftLimit(SoftLimitDirection.kForward, 155);
        wristMotor.setSoftLimit(SoftLimitDirection.kReverse, -120);

        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

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
        // if(speed > 0){
        // wristMotor.set(speed);
        // } else {
        // if(!atLimitSensor()){
        // wristMotor.set(speed);
        // } else {
        // wristMotor.set(0);
        // }
        // }
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

    public void setWristFeedForward(double dbks, double dbkg, double dbkv, double dbka) {
        if (kS != dbks || kG != dbkg || kV != dbkv || kA != dbka) {
            kS = dbks;
            kG = dbkg;
            kV = dbkv;
            kA = dbka;
            wristFeedforward = new ArmFeedforward(kS, kG, kV, kA);
        }
    }

    public void setPidController(double p, double i, double d, double ff) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
    }

    public boolean isMoving() {
        return wristMotor.get() != 0.0;
    }

    public void periodic() {

    }

    public void testPeriodic() {
        if (SmartDashboard.getBoolean("Toggle open loop wrist control", false)) {
            SmartDashboard.putNumber("Wrist Speed", OI.getInstance().getArmSpeed());
            setPercentOutput(OI.getInstance().getArmSpeed());
            arbitraryFF = 0;
        } else if (SmartDashboard.getBoolean("Toggle wrist PID tuning mode", false)) {
            setPidController(SmartDashboard.getNumber("Wrist P", WristConstants.kP),
                    SmartDashboard.getNumber("Wrist I", WristConstants.kI),
                    SmartDashboard.getNumber("Wrist D", WristConstants.kD),
                    SmartDashboard.getNumber("Wrist FF", WristConstants.kFF));

            setWristFeedForward(SmartDashboard.getNumber("Wrist kS", Constants.WristConstants.kSVolts),
                    SmartDashboard.getNumber("Wrist kG", WristConstants.kSVolts),
                    SmartDashboard.getNumber("Wrist kV", WristConstants.kVVoltSecondPerRad),
                    SmartDashboard.getNumber("Wrist kA", WristConstants.kAVoltSecondSquaredPerRad));

            setPosition(SmartDashboard.getNumber("Wrist PID setpoint (deg)", 0));

        }
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
}