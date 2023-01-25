package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class TestPath extends SequentialCommandGroup{

    public TestPath(Pose2d initialPose, PPSwerveControllerCommand part1) {
        addCommands(
                new InstantCommand(() -> Drivetrain.getInstance().resetOdometry(initialPose)),
                part1);

    }

    
}
