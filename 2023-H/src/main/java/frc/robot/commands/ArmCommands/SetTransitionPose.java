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
        new ParallelCommandGroup(new SetShoulderPosition(ShoulderConstants.kShoulderTransitionAngle));
        addRequirements(arm);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if(arm.getShoulderPosition() == ShoulderConstants.kShoulderTransitionAngle) 
            return true;
        else
            return false;
    }
}
