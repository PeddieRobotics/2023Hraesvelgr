package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private static Claw instance;
    private CANSparkMax clawMotor;
    private DigitalInput coneSensor, cubeSensor;

    private RollingAverage clawCurrentAverage;
    
    public enum ClawState {EMPTY, CUBE, CONE};
    private ClawState state; // Our current best estimation of the intake's state with respect to game pieces

    private SendableChooser<String> gamepieceChooser;

    private double cubeIntakeSpeed, coneIntakeSpeed, cubeOuttakeSpeed, coneOuttakeSpeed;

    public Claw() {
        clawMotor = new CANSparkMax(RobotMap.kClawMotor, MotorType.kBrushless);
        clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
        clawMotor.setIdleMode(IdleMode.kCoast);

        clawCurrentAverage = new RollingAverage();

        // coneSensor = new DigitalInput(RobotMap.kClawConeSensor);
        // cubeSensor = new DigitalInput(RobotMap.kClawCubeSensor);

        cubeIntakeSpeed = ClawConstants.kCubeIntakeSpeed;
        coneIntakeSpeed = ClawConstants.kConeIntakeSpeed;
        cubeOuttakeSpeed = ClawConstants.kCubeOuttakeSpeed;
        coneOuttakeSpeed = ClawConstants.kConeOuttakeSpeed;

        gamepieceChooser = new SendableChooser<String>();
        gamepieceChooser.addOption("Cone", "Cone");
        gamepieceChooser.addOption("Cube", "Cube");
        gamepieceChooser.addOption("None", "None");
        gamepieceChooser.setDefaultOption("None", "None");

        SmartDashboard.putData(gamepieceChooser);

        clawCurrentAverage = new RollingAverage();

        state = ClawState.EMPTY;
    }
    
    @Override
    public void periodic(){
        clawCurrentAverage.add(clawMotor.getOutputCurrent());
    }

    public static Claw getInstance(){
        if(instance == null){
            instance = new Claw();
        }
        return instance;
    }

    public ClawState getState() {
        return state;
    }

    public void setState(ClawState state) {
        this.state = state;
    }

    public void setSpeed(double clawSpeed){
        clawMotor.set(clawSpeed);
    }

    public boolean hasCone(){
        return state == ClawState.CONE;
        // return gamepieceChooser.getSelected().equals("Cone"); // temporary until we get banner sensors installed
        // return !coneSensor.get();
    }

    public boolean hasCube(){
        return state == ClawState.CUBE;
        // return gamepieceChooser.getSelected().equals("Cube"); // temporary until we get banner sensor installed
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
        setSpeed(cubeIntakeSpeed);
    }

    public void intakeCone(){
        setSpeed(coneIntakeSpeed);
    }

    public void outtakeCube(){
        setSpeed(cubeOuttakeSpeed);
    }

    public void outtakeCone(){
        setSpeed(coneOuttakeSpeed);
    }

    public double getMotorTemperature() {
        return clawMotor.getMotorTemperature();
    }

    public double getSpeed() {
        return clawMotor.get();
    }

    public double getOutputCurrent() {
        return clawMotor.getOutputCurrent();
    }

    public double getVoltage(){
        return clawMotor.getBusVoltage();
    }

}