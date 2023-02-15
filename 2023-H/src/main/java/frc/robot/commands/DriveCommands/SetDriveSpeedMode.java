package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.OI;
import frc.robot.utils.OI.DriveSpeedMode;

public class SetDriveSpeedMode extends CommandBase {
    private OI oi;
    private DriveSpeedMode mode;

    public SetDriveSpeedMode(DriveSpeedMode mode) {
        this.mode = mode;

        oi = OI.getInstance();
    }

    @Override
    public void initialize() {
        oi.setDriveSpeedMode(mode);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean isInterrupted) {
        oi.setDriveSpeedMode(DriveSpeedMode.NORMAL);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}