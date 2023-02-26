package frc.robot.commands.DriveCommands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockDrivetrain extends CommandBase{

    private Drivetrain drivetrain;

    public LockDrivetrain(){
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.lock();
    }

    //Called once the command ends or is interrupted
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules(); // Unlock/reset swerve modules when toggled off
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        return false;
    }
}
