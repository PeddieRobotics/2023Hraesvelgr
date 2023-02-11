package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class SetLevelThreePose extends CommandBase{
    private Arm arm;
    private Claw claw;

    public SetLevelThreePose() {
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        if(claw.hasCone())
            new ParallelCommandGroup(new SetShoulderPosition(ShoulderConstants.kShoulderLevelThreeConeAngle), new SetWristPosition(WristConstants.kWristLevelThreeConeAngle));
        else 
            new ParallelCommandGroup(new SetShoulderPosition(ShoulderConstants.kShoulderLevelThreeCubeLobAngle), new SetWristPosition(WristConstants.kWristLevelThreeCubeLobAngle));
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
        if((arm.getShoulderPosition() == ShoulderConstants.kShoulderLevelThreeConeAngle && arm.getWristPosition() == WristConstants.kWristLevelThreeConeAngle) || (arm.getShoulderPosition() == ShoulderConstants.kShoulderLevelThreeCubeLobAngle && arm.getWristPosition() == WristConstants.kWristLevelThreeCubeLobAngle))
            return true;
        else
            return false;
    }
}