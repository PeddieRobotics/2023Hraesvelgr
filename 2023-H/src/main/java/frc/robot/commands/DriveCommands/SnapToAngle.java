package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SnapToAngle extends CommandBase{
    private Drivetrain drivetrain;

    public SnapToAngle(){
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);;
    }

    @Override
    public void execute(){
        drivetrain.setSnapped(true);
    }

    //Called once the command ends or is interrupted
    public void end(boolean interrupted) {
        drivetrain.setSnapped(false);
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        return false;
    }
}
