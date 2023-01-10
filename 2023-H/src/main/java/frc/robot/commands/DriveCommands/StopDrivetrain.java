package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class StopDrivetrain extends CommandBase {
    private Drivetrain drivetrain;

    public StopDrivetrain() {
        drivetrain = Drivetrain.getInstance();
    }

    @Override
    public void initialize() {
        drivetrain.stopSwerveModules();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean isInterrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}