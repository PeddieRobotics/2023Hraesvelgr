package frc.robot.commands.DriveCommands;

import frc.robot.Robot;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockDrivetrain extends CommandBase{

    private Drivetrain drivetrain;

    public LockDrivetrain(){
        Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        drivetrain.lock();
    }

    //Called once the command ends or is interrupted
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        return true; //Ends when we are in a specific interval
    }
}
