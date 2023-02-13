package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private static Claw instance;
    private CANSparkMax clawMotor;
    private DigitalInput coneSensor, cubeSensor;

    public Claw() {
        clawMotor = new CANSparkMax(RobotMap.kClawMotor, MotorType.kBrushed);
        clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
        clawMotor.setIdleMode(IdleMode.kBrake);

        coneSensor = new DigitalInput(RobotMap.kClawConeSensor);
        cubeSensor = new DigitalInput(RobotMap.kClawCubeSensor);
    }
    
    @Override
    public void periodic(){
    }

    public void testPeriodic(){
        
    }

    public static Claw getInstance(){
        if(instance == null){
            instance = new Claw();
        }
        return instance;
    }

    public void setSpeed(double clawSpeed){
        clawMotor.set(clawSpeed);
    }

    public boolean hasCone(){
        return !coneSensor.get();
    }

    public boolean hasCube(){
        return !cubeSensor.get();
    }

    public double getClawSpeed(){
        return clawMotor.get();
    }

    public void stopClaw(){
        clawMotor.set(0);
    }

    public void intakeCube(){
        setSpeed(ClawConstants.kCubeIntakeSpeed);
    }

    public void intakeCone(){
        setSpeed(ClawConstants.kConeIntakeSpeed);
    }

    public void outtakeCube(){
        setSpeed(ClawConstants.kCubeOuttakeSpeed);
    }

    public void outtakeCone(){
        setSpeed(ClawConstants.kConeOuttakeSpeed);
    }

    public boolean isIntaking(){
        return clawMotor.get() != 0.0;
    }

    public double getCurrent(){
        return clawMotor.getOutputCurrent();
    }
}