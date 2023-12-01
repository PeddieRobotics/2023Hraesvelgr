package frc.robot.commands.DriveCommands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class StraightenDrivetrain extends Command{

    private Drivetrain drivetrain;
    private double initialTime;

    public StraightenDrivetrain(){
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.straighten();
        initialTime = Timer.getFPGATimestamp();
    }

    // Called once the command ends or is interrupted
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished(){
        return Timer.getFPGATimestamp() - initialTime > 0.25;
    }
}

