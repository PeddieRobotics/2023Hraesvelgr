package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class SetHumanPlayerPose extends CommandBase{
    private Arm arm;
    private Claw claw;

    public SetHumanPlayerPose() {
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        if(claw.hasCone())
            new ParallelCommandGroup(new SetShoulderPosition(ShoulderConstants.kShoulderHumanPlayerConeAngle), new SetWristPosition(WristConstants.kWristHumanPlayerConeAngle));
        else 
            new ParallelCommandGroup(new SetShoulderPosition(ShoulderConstants.kShoulderHumanPlayerCubeAngle), new SetWristPosition(WristConstants.kWristHumanPlayerCubeAngle));
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
        if((arm.getShoulderPosition() == ShoulderConstants.kShoulderHumanPlayerConeAngle && arm.getWristPosition() == WristConstants.kWristHumanPlayerConeAngle) || (arm.getShoulderPosition() == ShoulderConstants.kShoulderHumanPlayerCubeAngle && arm.getWristPosition() == WristConstants.kWristHumanPlayerCubeAngle))
            return true;
        else
            return false;
    }
}