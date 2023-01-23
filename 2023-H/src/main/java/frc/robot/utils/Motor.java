package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;

/**
 * For motors that don't constantly spin like the intake.
 */
public abstract class Motor {
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

    protected abstract int getMotorCanId();

    protected abstract String getPrefixedString(String baseString);

    protected Motor() {
        sparkMax = new CANSparkMax(getMotorCanId(), MotorType.kBrushless); // TODO: no idea what brushless means
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
        SmartDashboard.putNumber("Arbitrary FF (Arm FF)", feedforward);

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
        SmartDashboard.putNumber(getPrefixedString("kS"), kSVolts);
        SmartDashboard.putNumber(getPrefixedString("kG"), kGVolts);
        SmartDashboard.putNumber(getPrefixedString("kV"), kVVoltSecondPerRad);
        SmartDashboard.putNumber(getPrefixedString("kA"), kAVoltSecondSquaredPerRad);

        // PID controller parameters
        SmartDashboard.putNumber(getPrefixedString("P"), kP);
        SmartDashboard.putNumber(getPrefixedString("I"), kI);
        SmartDashboard.putNumber(getPrefixedString("D"), kD);
        SmartDashboard.putNumber(getPrefixedString("FF"), kFF);

        SmartDashboard.putBoolean(getPrefixedString("toggle pid active"), false);

        SmartDashboard.putNumber(getPrefixedString("speed % setpoint"), 0.0);
        SmartDashboard.putNumber(getPrefixedString("angle setpoint"), 0.0);
        SmartDashboard.putNumber(getPrefixedString("velocity setpoint"), 0.0);

    }
}
