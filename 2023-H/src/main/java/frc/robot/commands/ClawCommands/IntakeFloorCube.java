package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class IntakeFloorCube extends CommandBase{
    private Blinkin blinkin;
    private Claw claw;
    private double initialTime, currentTime;
    private boolean hasCone, hasCube;

    public IntakeFloorCube(){
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
        else if(!claw.isEitherSensor()){
            hasCube = false;
        }

        if(claw.isBackSensor() && !hasCone){
            hasCone = true;
            initialTime = Timer.getFPGATimestamp();
        }
        else if(!claw.isBackSensor()){
            hasCone = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopMonitoringCurrent();
        claw.classifyGamepiece();
        if(claw.hasGamepiece()){
            Blinkin.getInstance().success();
        }
    }

    @Override
    public boolean isFinished() {
        return (hasCube && (currentTime - initialTime) > 0.1) || (hasCone && (currentTime - initialTime) > 0.1);
    }

    
}
