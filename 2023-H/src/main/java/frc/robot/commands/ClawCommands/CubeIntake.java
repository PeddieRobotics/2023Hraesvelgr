package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArmCommands.SetLevelOnePose;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Constants.ClawConstants;

public class CubeIntake extends CommandBase{
    private Claw claw;

    public CubeIntake(){
        claw = Claw.getInstance();

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        // If we have any gamepiece, this command functions as a L1 pose button
        if(claw.hasCone() || claw.hasCube()){
            CommandScheduler.getInstance().schedule(new SetLevelOnePose());
        }
        // If we don't have any game pieces, we must be intaking a cube
        else{
            claw.intakeCube();
        }
        
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
    }

    @Override
    public boolean isFinished() {
        return claw.hasCube();
    }

    
}
