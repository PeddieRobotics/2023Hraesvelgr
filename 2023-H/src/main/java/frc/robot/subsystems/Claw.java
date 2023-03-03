package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.RobotMap;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private static Claw instance;
    private CANSparkMax clawMotor;
    private DigitalInput backSensor, frontSensor;
    private boolean useSensors;

    public enum ClawState {EMPTY, INTAKING, CUBE, CONE};
    private ClawState state; // Our current best estimation of the intake's state with respect to game pieces

    private SendableChooser<String> gamepieceChooser;

    private double cubeIntakeSpeed, coneIntakeSpeed, cubeOuttakeSpeed, coneOuttakeSpeed;

    public Claw() {
        clawMotor = new CANSparkMax(RobotMap.kClawMotor, MotorType.kBrushless);
        clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
        clawMotor.setIdleMode(IdleMode.kCoast);

        backSensor = new DigitalInput(RobotMap.kClawBackSensor);
        frontSensor = new DigitalInput(RobotMap.kClawFrontSensor);
        useSensors = true;

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

        state = ClawState.EMPTY;
    }
    
    @Override
    public void periodic(){
        if(useSensors){
            if(isFrontSensor() && isBackSensor()){
                state = ClawState.CONE;
            }
            else if(isFrontSensor() && !isBackSensor()){
                state = ClawState.CUBE;
            }
            else if(state != ClawState.INTAKING && (!isFrontSensor() && !isBackSensor())){
                state = ClawState.EMPTY;
            }
        }
        
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

    public boolean isBackSensor(){
        return !backSensor.get();
    }

    public boolean isFrontSensor(){
        return !frontSensor.get();
    }

    public boolean hasCone(){
        return state == ClawState.CONE;
    }

    public boolean hasCube(){
        return state == ClawState.CUBE;
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
        return clawMotor.getAppliedOutput()*100;
    }

        
    public boolean isUseSensors() {
        return useSensors;
    }

    public void setUseSensors(boolean useSensors) {
        this.useSensors = useSensors;
    }


}