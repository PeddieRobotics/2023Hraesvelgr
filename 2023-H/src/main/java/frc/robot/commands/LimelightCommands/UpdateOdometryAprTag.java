package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

public class UpdateOdometryAprTag extends CommandBase {
    private final Limelight limelight;
    private final Drivetrain drivetrain;

    //private static UpdateLogs updatelogs = UpdateLogs.getInstance();


    public UpdateOdometryAprTag() {
        limelight = LimelightBack.getInstance();
        drivetrain = Drivetrain.getInstance();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetRobotPoseAndGyro(limelight.getBotpose());
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
       return true;
    }
}