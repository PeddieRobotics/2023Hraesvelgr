package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class IntakeCube extends CommandBase{
    private Blinkin blinkin;
    private Claw claw;
    private double initialTime, currentTime;
    private boolean hasCone, hasCube;

    public IntakeCube(){
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();
        addRequirements(claw);

        initialTime = 0.0;
        currentTime = 0.0;
    }

    @Override
    public void initialize() {
        blinkin.intakingCube();
        claw.setState(ClawState.INTAKING_CUBE);     
        hasCone = false;
        hasCube = false;
        claw.intakeCube();
        claw.startMonitoringCurrent();
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
        if(claw.analyzeCurrentForCube() && !hasCube){
            hasCube = true;
            initialTime = Timer.getFPGATimestamp();
        }
        else if(!claw.hasGamepiece()){
            hasCube = false;
        }

        if(claw.hasCone() && !hasCone){
            hasCone = true;
            initialTime = Timer.getFPGATimestamp();
        }
        else if(!claw.hasCone()){
            hasCone = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopMonitoringCurrent();

        if(!interrupted){
            if(claw.hasCube()){
                blinkin.success();
                claw.setSpeed(ClawConstants.kCubeHoldSpeed);
            }
            else if(claw.hasCone()){
                blinkin.success();
                claw.stopClaw();
                claw.monitorNewConeIntake();
            }
            else{
                claw.setState(ClawState.EMPTY);     
                blinkin.failure();
                claw.stopClaw();
            }
        }
        else{
            if(claw.hasCube()){
                blinkin.success();
                claw.setSpeed(ClawConstants.kCubeHoldSpeed);
            }
            else if(claw.hasCone()){
                blinkin.success();
                claw.stopClaw();
                claw.monitorNewConeIntake();
            }
            else{
                claw.setState(ClawState.EMPTY);   
                blinkin.returnToRobotState();  
                claw.stopClaw();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return (hasCube && (currentTime - initialTime) > 0.2) || (hasCone && (currentTime - initialTime) > 0.2);
    }

    
}
