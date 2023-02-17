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
import frc.robot.utils.Constants.ShoulderConstants;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Shoulder{

    private static Shoulder shoulder;

    private CANSparkMax shoulderMotorMaster, shoulderMotorFollower;

    private SparkMaxPIDController pidController;
    private ArmFeedforward shoulderFeedforward;

    private double kS, kG, kV, kA, arbitraryFF;

    public Shoulder() {
        shoulderMotorMaster = new CANSparkMax(RobotMap.kShoulderMotorMaster, MotorType.kBrushless);
        shoulderMotorMaster.setSmartCurrentLimit(ShoulderConstants.kMaxCurrent);

        shoulderMotorFollower = new CANSparkMax(RobotMap.kShoulderMotorFollower, MotorType.kBrushless);
        shoulderMotorFollower.setSmartCurrentLimit(ShoulderConstants.kMaxCurrent);

        shoulderMotorFollower.follow(shoulderMotorMaster);

        shoulderMotorMaster.setIdleMode(IdleMode.kCoast);
        shoulderMotorFollower.setIdleMode(IdleMode.kCoast);

        pidController = shoulderMotorMaster.getPIDController();

        kS = ShoulderConstants.kSVolts;
        kG = ShoulderConstants.kGVolts;

        kV = ShoulderConstants.kVVoltSecondPerRad;
        kA = ShoulderConstants.kAVoltSecondSquaredPerRad;
        shoulderFeedforward = new ArmFeedforward(ShoulderConstants.kSVolts, ShoulderConstants.kGVolts, ShoulderConstants.kVVoltSecondPerRad, ShoulderConstants.kAVoltSecondSquaredPerRad);

        shoulderMotorMaster.setInverted(true);
        shoulderMotorFollower.setInverted(true);

        shoulderMotorMaster.getEncoder().setPositionConversionFactor(ShoulderConstants.kShoulderEncoderConversionFactor);
        shoulderMotorFollower.getEncoder().setPositionConversionFactor(ShoulderConstants.kShoulderEncoderConversionFactor);
        setEncoder(-75.0);

        shoulderMotorMaster.setClosedLoopRampRate(0.01);

        shoulderMotorMaster.setSoftLimit(SoftLimitDirection.kForward, 155);
        shoulderMotorMaster.setSoftLimit(SoftLimitDirection.kReverse, -75);
        
        shoulderMotorFollower.setSoftLimit(SoftLimitDirection.kForward, 155);
        shoulderMotorFollower.setSoftLimit(SoftLimitDirection.kReverse, -75);

        shoulderMotorMaster.enableSoftLimit(SoftLimitDirection.kForward, true);
        shoulderMotorMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);
        shoulderMotorFollower.enableSoftLimit(SoftLimitDirection.kForward, true);
        shoulderMotorFollower.enableSoftLimit(SoftLimitDirection.kReverse, true);
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

    public void stopShoulder(){
        shoulderMotorMaster.set(0);
        shoulderMotorFollower.set(0);
    }

    public void setPosition(double setpointDeg) {
        arbitraryFF = shoulderFeedforward.calculate(Math.toRadians(setpointDeg), 0);

        pidController.setReference(setpointDeg, ControlType.kPosition, 0, arbitraryFF);
    }

    public double getArbitraryFF(){
        return arbitraryFF;
    }

    public void setAngleSmartMotion(double setpoint){
        pidController.setReference(setpoint, ControlType.kSmartMotion);
    }

    public void resetEncoder() {
        setEncoder(0);
    }

    public void setEncoder(double newEncoderValue){
        shoulderMotorMaster.getEncoder().setPosition(newEncoderValue);
        shoulderMotorFollower.getEncoder().setPosition(newEncoderValue);
    }

    public void setPercentOutput(double speed) {
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

    public boolean isMoving(){
        return shoulderMotorMaster.get() != 0.0;
    }

    public void disablePIDController(){
        setShoulderFeedforward(0, 0, 0, 0);
        setPidController(0, 0, 0, 0, 0);
    }

    public void setShoulderFeedforward(double dbks, double dbkg, double dbkv, double dbka){
        kS = dbks;
        kG = dbkg;
        kV = dbkv;
        kA = dbka;
        shoulderFeedforward = new ArmFeedforward(kS, kG, kV, kA);
    }

    public void setPidController(double p, double i, double d, double ff, double izone){
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
        pidController.setIZone(izone);
    }

    public void periodic() {
    }

    public void testPeriodic(){
        

        if(SmartDashboard.getBoolean("Toggle open loop shoulder control", false)){
                setPercentOutput(OI.getInstance().getArmSpeed());
                arbitraryFF = 0;
        }
        else if(SmartDashboard.getBoolean("Toggle shoulder PID tuning mode", false)){
            setPidController(SmartDashboard.getNumber("Shoulder P", Constants.ShoulderConstants.kP),
                SmartDashboard.getNumber("Shoulder I", Constants.ShoulderConstants.kI),
                SmartDashboard.getNumber("Shoulder D", Constants.ShoulderConstants.kD),
                SmartDashboard.getNumber("Shoulder FF", Constants.ShoulderConstants.kFF),
                SmartDashboard.getNumber("Shoulder IZone", Constants.ShoulderConstants.kIz));

            setShoulderFeedforward(SmartDashboard.getNumber("Shoulder kS", Constants.ShoulderConstants.kSVolts),
                SmartDashboard.getNumber("Shoulder kG", Constants.ShoulderConstants.kSVolts), 
                SmartDashboard.getNumber("Shoulder kV", Constants.ShoulderConstants.kVVoltSecondPerRad),
                SmartDashboard.getNumber("Shoulder kA", Constants.ShoulderConstants.kAVoltSecondSquaredPerRad));

            double shoulderPIDSetpoint = SmartDashboard.getNumber("Shoulder PID setpoint (deg)",0);
            if(shoulderPIDSetpoint >= -75.0 && shoulderPIDSetpoint <= 145.0 && SmartDashboard.getBoolean("Execute", false)){
                setPosition(shoulderPIDSetpoint);
            }
        }

    }

    public void setMode(IdleMode mode){
        shoulderMotorMaster.setIdleMode(mode);
        shoulderMotorFollower.setIdleMode(mode);
    }

    public static Shoulder getInstance() {
        if(shoulder == null){
            shoulder = new Shoulder();
        }
        return shoulder;
    }
}