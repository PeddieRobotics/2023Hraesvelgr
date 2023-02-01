package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMapH;
import frc.robot.utils.Constants.ShoulderConstants;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Shoulder extends SubsystemBase{

    private static Shoulder shoulder;

    private CANSparkMax shoulderMotorMaster, shoulderMotorFollower;

    private SparkMaxPIDController pidController;

    private ArmFeedforward armFeedforward;

    private double kS, kG, kV, kA;


    public Shoulder() {
        shoulderMotorMaster = new CANSparkMax(RobotMapH.kShoulderMotorMaster, MotorType.kBrushless);
        shoulderMotorFollower.setSmartCurrentLimit(ShoulderConstants.kMaxCurrent);

        shoulderMotorFollower = new CANSparkMax(RobotMapH.kShoulderMotorFollower, MotorType.kBrushless);
        shoulderMotorFollower.setSmartCurrentLimit(ShoulderConstants.kMaxCurrent);

        shoulderMotorFollower.follow(shoulderMotorMaster);

        shoulderMotorMaster.setIdleMode(IdleMode.kBrake);
        shoulderMotorFollower.setIdleMode(IdleMode.kBrake);

        pidController = shoulderMotorMaster.getPIDController();

        kS = ShoulderConstants.kSVolts;
        kG = ShoulderConstants.kGVolts;

        kV = ShoulderConstants.kVVoltSecondPerRad;
        kA = ShoulderConstants.kAVoltSecondSquaredPerRad;
        armFeedforward = new ArmFeedforward(ShoulderConstants.kSVolts, ShoulderConstants.kGVolts, ShoulderConstants.kVVoltSecondPerRad, ShoulderConstants.kAVoltSecondSquaredPerRad);
    }

    public double getAngle() {
        return shoulderMotorMaster.getEncoder().getPosition();
    }

    public void setSpeed(double speed){
        shoulderMotorMaster.set(speed);
    }

    public double getSpeed(){
        return shoulderMotorMaster.get();
    }

    public double getOutputCurrent(){
        return shoulderMotorMaster.getOutputCurrent();
    }

    public void setAnglePosition(double setpoint) {
        double feedforward = armFeedforward.calculate(Math.toRadians(90.0 - setpoint), 0);
        SmartDashboard.putNumber("shoulder Arbitrary FF (Arm FF)", feedforward);

        pidController.setReference(setpoint, ControlType.kPosition, 0, feedforward);
    }

    public void setAngleSmartMotion(double setpoint){
        pidController.setReference(setpoint, ControlType.kSmartMotion);
    }

    public void setVelocity(double setpoint) {
        pidController.setReference(setpoint, ControlType.kVelocity);
    }

    public void resetEncoder() {
        shoulderMotorMaster.getEncoder().setPosition(0);
    }

    public void setMotor(double speed) {
        shoulderMotorMaster.set(speed);
    }

    public double getMotorTemperature(){
        return shoulderMotorMaster.getMotorTemperature();
    }

    public double getPosition(){
        return shoulderMotorMaster.getEncoder().getPosition();
    }

    public double getVelocity(){
        return shoulderMotorMaster.getEncoder().getVelocity();
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



    public void setPidController(double p, double i, double d, double ff, boolean pidActive){
        if(pidActive){
            pidController.setP(p);
            pidController.setI(i);
            pidController.setD(d);
            pidController.setFF(ff);
        }
    }

    @Override
    public void periodic() {
    }

    public static Shoulder getInstance() {
        if(shoulder == null){
            shoulder = new Shoulder();
        }
        return shoulder;
    }
}