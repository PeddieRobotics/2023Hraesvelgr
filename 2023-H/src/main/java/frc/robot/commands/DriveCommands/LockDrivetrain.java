package frc.robot.commands.DriveCommands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockDrivetrain extends CommandBase{

    private Drivetrain drivetrain;
    private boolean success;

    public LockDrivetrain(){
        drivetrain=Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        //if(drivetrain.getSpeed() < 0.5){
            drivetrain.lock();
            success = true;
        //}
    }

    //Called once the command ends or is interrupted
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules(); // Unlock/reset swerve modules when toggled off
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        if(success){
            return false; // Wait to be toggled off
        }
        else{
            return true; // If we weren't able to lock the drivetrain, return to normal driving
        }
    }
}
