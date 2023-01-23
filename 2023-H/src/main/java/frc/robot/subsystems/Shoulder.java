package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Shoulder {
    final int motorCanId = 1;
    final int kMaxCurrent = 10;

    final double kP = 0.01;
    final double kI = 0.00000035;
    final double kD = 0.0;
    final double kIz = 0.0;
    final double kFF = 0.0;
    final double kAngleMin = -30;
    final double kAngleMax = 120;

    final double kSVolts = 0.0;
    final double kGVolts = -0.3875;
    final double kVVoltSecondPerRad = 0.0;
    final double kAVoltSecondSquaredPerRad = 0.0;

    final double kMaxVelocityRadPerSecond = 1;
    final double kMaxAccelerationRadPerSecSquared = 3;

    final double kEncoderPPR = 205.12;
    final double kEncoderDistancePerPulse = 360.0 / kEncoderPPR;

    final double kSpeedMultiplier = 0.5;
    final double kTurnMultiplier = 0.25;

    CANSparkMax sparkMax;

    SparkMaxPIDController pidController;

    ArmFeedforward armFeedforward;

    double kS, kG, kV, kA;

    protected Shoulder() {
        sparkMax = new CANSparkMax(motorCanId, MotorType.kBrushless); // TODO: no idea what brushless means
        sparkMax.setSmartCurrentLimit(kMaxCurrent);

        pidController = sparkMax.getPIDController();

        kS = kSVolts;
        kG = kGVolts;

        kV = kVVoltSecondPerRad;
        kA = kAVoltSecondSquaredPerRad;
        armFeedforward = new ArmFeedforward(kSVolts, kGVolts, kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);
    }

    public double getAngle() {
        return sparkMax.getEncoder().getPosition();
    }

    public void setAngle(double setpoint) {
        double feedforward = armFeedforward.calculate(Math.toRadians(90.0 - setpoint), 0);
        SmartDashboard.putNumber("shoulder Arbitrary FF (Arm FF)", feedforward);

        pidController.setReference(setpoint, ControlType.kPosition, 0, feedforward);
    }

    public void setVelocity(double setpoint) {
        pidController.setReference(setpoint, ControlType.kVelocity);
    }

    public void resetEncoder() {
        sparkMax.getEncoder().setPosition(0);
        // SmartDashboard.putBoolean("is encoder reset", true);
    }

    public void setMotor(double speed) {
        sparkMax.set(speed);
    }

    public void putSmartDashboardOverrides() {
        // ArmFeedforward parameters
        SmartDashboard.putNumber("shoulder kS", kSVolts);
        SmartDashboard.putNumber("shoulder kG", kGVolts);
        SmartDashboard.putNumber("shoulder kV", kVVoltSecondPerRad);
        SmartDashboard.putNumber("shoulder kA", kAVoltSecondSquaredPerRad);

        // PID controller parameters
        SmartDashboard.putNumber("shoulder P", kP);
        SmartDashboard.putNumber("shoulder I", kI);
        SmartDashboard.putNumber("shoulder D", kD);
        SmartDashboard.putNumber("shoulder FF", kFF);

        SmartDashboard.putBoolean("shoulder toggle pid active", false);

        SmartDashboard.putNumber("shoulder speed % setpoint", 0.0);
        SmartDashboard.putNumber("shoulder angle setpoint", 0.0);
        SmartDashboard.putNumber("shoulder velocity setpoint", 0.0);
    }

    public void updateFromDashboard() {

        // Update dashboard with robot info during test mode
        SmartDashboard.putNumber("shoulder encoder", sparkMax.getEncoder().getPosition());
        SmartDashboard.putNumber("shoulder angle", getAngle());
        SmartDashboard.putNumber("shoulder velocity", sparkMax.getEncoder().getVelocity());
        SmartDashboard.putNumber("shoulder Current", sparkMax.getOutputCurrent());
        SmartDashboard.putNumber("shoulder Bus Voltage", sparkMax.getBusVoltage());
        SmartDashboard.putNumber("shoulder temperature", sparkMax.getMotorTemperature());

        // Update kS, kG, kV, kA for ArmFeedforward
        double dbks = SmartDashboard.getNumber("shoulder kS", kSVolts);
        double dbkg = SmartDashboard.getNumber("shoulder kG", kGVolts);
        double dbkv = SmartDashboard.getNumber("shoulder kV", kVVoltSecondPerRad);
        double dbka = SmartDashboard.getNumber("shoulder kA", kAVoltSecondSquaredPerRad);

        if (kS != dbks || kG != dbkg || kV != dbkv || kA != dbka) {
            kS = dbks;
            kG = dbkg;
            kV = dbkv;
            kA = dbka;
            armFeedforward = new ArmFeedforward(kS, kG, kV, kA);
        }

        // Update PID controller
        pidController.setP(SmartDashboard.getNumber("shoulder P", kP));
        pidController.setI(SmartDashboard.getNumber("shoulder I", kI));
        pidController.setD(SmartDashboard.getNumber("shoulder D", kD));
        pidController.setFF(SmartDashboard.getNumber("shoulder FF", kFF));
    }

    public void periodic() {
        SmartDashboard.putNumber("shoulder encoder", sparkMax.getEncoder().getPosition());
        SmartDashboard.putNumber("shoulder angle", getAngle());
        SmartDashboard.putNumber("shoulder velocity", sparkMax.getEncoder().getVelocity());
        SmartDashboard.putNumber("shoulder Motor Current", sparkMax.getOutputCurrent());
        SmartDashboard.putNumber("shoulder Motor Bus Voltage", sparkMax.getBusVoltage());
        SmartDashboard.putNumber("shoulder Motor temperature", sparkMax.getMotorTemperature());
    }

    public CANSparkMax getMotor() {
        return sparkMax;
    }
}