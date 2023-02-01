package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMapH;
import frc.robot.utils.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private static Claw instance;
    private CANSparkMax clawMotor;

    public Claw() {
        clawMotor = new CANSparkMax(RobotMapH.kClawMotor, MotorType.kBrushed);
        clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
        clawMotor.setIdleMode(IdleMode.kBrake);
    }
    
    @Override
    public void periodic(){
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