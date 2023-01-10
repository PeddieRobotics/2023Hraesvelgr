package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class StopDriveAndWait extends ParallelRaceGroup {
    public StopDriveAndWait(double seconds) {
        addCommands(
                new StopDrivetrain(),
                new WaitCommand(seconds));
    }


} 
