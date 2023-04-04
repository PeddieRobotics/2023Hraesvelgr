package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class IntakeCubeSingleSS extends CommandBase{
    private Blinkin blinkin;
    private Claw claw;
    private double initialTime, currentTime;

    public IntakeCubeSingleSS(){
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
        claw.intakeCube();
 
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

    }

    @Override
    public void end(boolean interrupted) {
        claw.classifyGamepiece();
        if(claw.hasGamepiece()){
            Blinkin.getInstance().success();
        }
    }

    @Override
    public boolean isFinished() {
        return claw.isEitherSensor() && (currentTime - initialTime) > 0.5;
    }

    
}
