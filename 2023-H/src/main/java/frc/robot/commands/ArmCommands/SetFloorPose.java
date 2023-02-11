package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class SetFloorPose extends CommandBase{
    private Arm arm;
    private Claw claw;

    public SetFloorPose() {
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        if(claw.hasCone())
            new ParallelCommandGroup(new SetShoulderPosition(ShoulderConstants.kShoulderFloorConeAngle), new SetWristPosition(WristConstants.kWristFloorConeAngle));
        else 
            new ParallelCommandGroup(new SetShoulderPosition(ShoulderConstants.kShoulderFloorCubeAngle), new SetWristPosition(WristConstants.kWristFloorCubeAngle));
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
        if((arm.getShoulderPosition() == ShoulderConstants.kShoulderFloorConeAngle && arm.getWristPosition() == WristConstants.kWristFloorConeAngle) || (arm.getShoulderPosition() == ShoulderConstants.kShoulderFloorCubeAngle && arm.getWristPosition() == WristConstants.kWristFloorCubeAngle))
            return true;
        else
            return false;
    }
}