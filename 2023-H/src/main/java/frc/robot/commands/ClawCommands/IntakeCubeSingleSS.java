package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class IntakeCubeSingleSS extends Command{
    private Blinkin blinkin;
    private Claw claw;
    private double initialTime, currentTime;
    private boolean blinkinSuccess;

    public IntakeCubeSingleSS(){
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();
        addRequirements(claw);

        initialTime = 0.0;
        currentTime = 0.0;

        blinkinSuccess = false;
    }

    @Override
    public void initialize() {
        blinkin.intakingCube();
        claw.setState(ClawState.INTAKING_CUBE);     
        claw.intakeCube();
 
        blinkinSuccess = false;
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
        if(!blinkinSuccess && claw.isEitherSensor()){
            Blinkin.getInstance().success();
            blinkinSuccess = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        claw.classifyGamepiece();
    }

    @Override
    public boolean isFinished() {
        return claw.isEitherSensor() && (currentTime - initialTime) > 0.5;
    }

    
}
