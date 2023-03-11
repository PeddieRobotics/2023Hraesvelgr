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
import frc.robot.utils.Constants.WristConstants;

public class Claw extends SubsystemBase {
    private static Claw instance;
    private CANSparkMax clawMotor;
    private DigitalInput backSensor, frontSensor;
    private boolean useSensors;

    public enum ClawState {
        EMPTY, INTAKING, CUBE, CONE
    };

    private ClawState state; // Our current best estimation of the intake's state with respect to game pieces

    private final LimelightFront limelightFront;
    private double coneAlignmentError;
    private boolean monitorNewConeIntake;
    private int newConeCounter;

    public Claw() {
        clawMotor = new CANSparkMax(RobotMap.kClawMotor, MotorType.kBrushless);
        clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
        clawMotor.setIdleMode(IdleMode.kCoast);

        backSensor = new DigitalInput(RobotMap.kClawBackSensor);
        frontSensor = new DigitalInput(RobotMap.kClawFrontSensor);
        useSensors = true;

        state = ClawState.EMPTY;

        limelightFront = LimelightFront.getInstance();
        coneAlignmentError = 0.0;
        monitorNewConeIntake = false;
        newConeCounter = 0;
    }

    @Override
    public void periodic() {
        if (useSensors) {
            if (isFrontSensor() && isBackSensor()) {
                state = ClawState.CONE;
            } else if (isFrontSensor() && !isBackSensor()) {
                state = ClawState.CUBE;
            } else if (state != ClawState.INTAKING && (!isFrontSensor() && !isBackSensor())) {
                state = ClawState.EMPTY;
            }
        }

        // If we've recently intaked a cone, and the arm is oriented such that we could
        // see the cone, start looking at it.
        if (monitorNewConeIntake && limelightFront.hasTarget() && Arm.getInstance().isWristAtAngle(WristConstants.kMonitorConeAlignmentAngle)) {
            updateConeAlignmentError();
            newConeCounter++;

            // If we have looked the cone for at least 240 ms, we've gotten enough of a
            // glimpse.
            if (newConeCounter > 4) {
                monitorNewConeIntake = false; // Stop looking at cone alignment
                newConeCounter = 0;
            }
        }

        SmartDashboard.putNumber("newConeCounter", newConeCounter);
        SmartDashboard.putNumber("coneAlignmentError", coneAlignmentError);
        SmartDashboard.putBoolean("monitorNewConeIntake", monitorNewConeIntake);

    }

    public static Claw getInstance() {
        if (instance == null) {
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

    public void setSpeed(double clawSpeed) {
        clawMotor.set(clawSpeed);
    }

    public boolean isBackSensor() {
        return !backSensor.get();
    }

    public boolean isFrontSensor() {
        return !frontSensor.get();
    }

    public boolean hasCone() {
        return state == ClawState.CONE;
    }

    public boolean hasCube() {
        return state == ClawState.CUBE;
    }

    public boolean hasGamepiece() {
        return hasCone() || hasCube();
    }

    public double getClawSpeed() {
        return clawMotor.get();
    }

    public void stopClaw() {
        clawMotor.set(0);
    }

    public void intakeCube() {
        setSpeed(ClawConstants.kCubeIntakeSpeed);
    }

    public void intakeCone() {
        setSpeed(ClawConstants.kConeIntakeSpeed);
    }

    public void outtakeCube() {
        ArmState armState = Arm.getInstance().getState();
        if(armState == ArmState.L1){
            setSpeed(ClawConstants.kCubeL1OuttakeSpeed);
        }
        else if(armState == ArmState.L2_CONE){
            setSpeed(ClawConstants.kCubeL2OuttakeSpeed);
        }
        else if(armState == ArmState.L3_CUBE_INVERTED){
            setSpeed(ClawConstants.kCubeL3InvertedOuttakeSpeed);
        }
        else if(armState == ArmState.L3_CUBE_FORWARD){
            setSpeed(ClawConstants.kCubeL3ForwardOuttakeSpeed);
        }
        else{
            setSpeed(ClawConstants.kConeL1OuttakeSpeed);
        }
    }

    public void outtakeCone() {
        ArmState armState = Arm.getInstance().getState();
        if(armState == ArmState.L1){
            setSpeed(ClawConstants.kConeL1OuttakeSpeed);
        }
        else if(armState == ArmState.L2_CONE){
            setSpeed(ClawConstants.kConeL2OuttakeSpeed);
        }
        else if(armState == ArmState.L3_CONE_INVERTED){
            setSpeed(ClawConstants.kConeL3InvertedOuttakeSpeed);
        }
        else if(armState == ArmState.L3_CONE_FORWARD){
            setSpeed(ClawConstants.kConeL3ForwardOuttakeSpeed);
        }
        else{
            setSpeed(ClawConstants.kConeL1OuttakeSpeed);
        }

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

    public double getVoltage() {
        return clawMotor.getAppliedOutput() * 100;
    }

    public boolean isUseSensors() {
        return useSensors;
    }

    public void setUseSensors(boolean useSensors) {
        this.useSensors = useSensors;
    }

    public double getConeAlignmentError() {
        return coneAlignmentError;
    }

    public void setConeAlignmentError(double coneAlignmentError) {
        this.coneAlignmentError = coneAlignmentError;
    }

    public void updateConeAlignmentError() {
        if (state != ClawState.CONE) {
            coneAlignmentError = 0.0;
        } else {
            coneAlignmentError = convertConeTXToAlignmentError(limelightFront.getTxAverage());
        }

    }

    public double convertConeTXToAlignmentError(double tx) {
        return tx / 3.5; // Use y=2/7 x as a simple linear regression (based on some quick empirical
                         // data), where y is robot tx to goal, x is cone tx compared to center of
                         // intake.
    }

    public void monitorNewConeIntake() {
        limelightFront.setPipeline(7);
        limelightFront.resetRollingAverages();
        monitorNewConeIntake = true;
    }

    public boolean isMonitorNewConeIntake(){
        return monitorNewConeIntake;
    }

    public void resetConeAlignmentError(){
        coneAlignmentError = 0;
    }
}
