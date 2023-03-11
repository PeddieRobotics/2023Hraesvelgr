package frc.robot.commands.LimelightCommands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

public class LocalizeWithLL extends CommandBase {
    private final Limelight limelightFront, limelightBack;
    private SwerveDrivePoseEstimator odometry;

    public LocalizeWithLL() {
        limelightFront = LimelightFront.getInstance();
        limelightBack = LimelightBack.getInstance();

    }

    @Override
    public void initialize() {
        odometry = Drivetrain.getInstance().getOdometry();
    }

    @Override
    public void execute() {
        limelightFront.forceAprilTagLocalization(odometry);
        limelightBack.forceAprilTagLocalization(odometry);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}