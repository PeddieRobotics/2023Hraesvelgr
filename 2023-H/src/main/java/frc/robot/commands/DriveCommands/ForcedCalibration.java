package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class ForcedCalibration extends Command {
    private Drivetrain drivetrain;
    private int cycles = 0;
    private static final int maxCycles = 2;

    public ForcedCalibration() {
        drivetrain = Drivetrain.getInstance();
   }

    @Override
    public void initialize() {
        drivetrain.setIsAdjusting(true);
        cycles = 0;
    }

    @Override
    public void execute() {
        cycles++;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setIsAdjusting(false);
    }

    @Override
    public boolean isFinished() {
        return cycles >= maxCycles;
    }
}