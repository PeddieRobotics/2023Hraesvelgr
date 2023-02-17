package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetFloorCubePose extends CommandBase{
    private Arm arm;
    private boolean hasStowed;

    public SetFloorCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        hasStowed = false;
    }

    @Override
    public void initialize() {
        arm.setWristPosition(WristConstants.kWristStowedAngle);
    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(-10)){
            arm.setShoulderPosition(ShoulderConstants.kShoulderStowedAngle);
        }

        if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderStowedAngle)){
            hasStowed = true;
            arm.setShoulderPosition(ShoulderConstants.kShoulderFloorCubeAngle);
        }

        if(hasStowed && arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorCubeAngle)){
            arm.setWristPosition(WristConstants.kWristFloorCubeAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorCubeAngle) && arm.isWristAtAngle(WristConstants.kWristFloorCubeAngle);
    }


}

