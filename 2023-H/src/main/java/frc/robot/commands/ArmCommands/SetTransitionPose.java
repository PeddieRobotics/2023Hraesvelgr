package frc.robot.commands.ArmCommands;

import javax.xml.stream.events.EndDocument;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ArmCommands.SetShoulderPosition;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SetTransitionPose extends CommandBase{
    private Arm arm;

    public SetTransitionPose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPosition(ShoulderConstants.kShoulderTransitionAngle);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderTransitionAngle);
    }
}
