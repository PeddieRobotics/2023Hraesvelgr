package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private static Claw instance;
    private CANSparkMax clawMotor;
    private DigitalInput coneSensor, cubeSensor;

    public Claw() {
        clawMotor = new CANSparkMax(RobotMap.kClawMotor, MotorType.kBrushless);
        clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
        clawMotor.setIdleMode(IdleMode.kCoast);

        coneSensor = new DigitalInput(RobotMap.kClawConeSensor);
        cubeSensor = new DigitalInput(RobotMap.kClawCubeSensor);
    }
    
    @Override
    public void periodic(){
        
    }

    public void testPeriodic(){
        // setSpeed(SmartDashboard.getNumber("OR: Claw speed", 0));
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
        return Shuffleboard.getInstance().hasCone(); // temporary until we get banner sensors installed
        // return !coneSensor.get();
    }

    public boolean hasCube(){
        return Shuffleboard.getInstance().hasCube(); // temporary until we get banner sensor installed
        // return !cubeSensor.get();
    }

    public boolean hasGamepiece(){
        return hasCone() || hasCube();
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