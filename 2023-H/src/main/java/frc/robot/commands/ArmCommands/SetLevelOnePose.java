package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SetLevelOnePose extends CommandBase{
    private Arm arm;

    public SetLevelOnePose() {
        arm = Arm.getInstance();
        new ParallelCommandGroup(new SetShoulderPosition(ShoulderConstants.kShoulderLevelOneAngle), new SetWristPosition(WristConstants.kWristLevelOneAngle));
        addRequirements(arm);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interupted){
    }

    @Override public boolean isFinished() {
        if(arm.getShoulderPosition() == ShoulderConstants.kShoulderLevelOneAngle && arm.getWristPosition() == WristConstants.kWristLevelOneAngle)
            return true;
        else 
            return false;
    }


}
