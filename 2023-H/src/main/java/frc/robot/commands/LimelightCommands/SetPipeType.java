package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;

public class SetPipeType extends CommandBase {
    private final LimelightBack limelightBack;
    private final int column;

    //private static UpdateLogs updatelogs = UpdateLogs.getInstance();


    public SetPipeType(int col) {
        limelightBack = LimelightBack.getInstance();
        column=col;
    }

    @Override
    public void initialize() {
        limelightBack.setPipelineType(column);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        limelightBack.setPipelineType(column);
    }

    @Override
    public boolean isFinished() {
       return true;
    }
}