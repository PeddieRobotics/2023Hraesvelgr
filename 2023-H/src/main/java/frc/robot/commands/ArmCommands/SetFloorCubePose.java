package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetFloorCubePose extends CommandBase{
    private Arm arm;
    private boolean shoulderStowed, shoulderStowing;

    public SetFloorCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        shoulderStowed = false;
        shoulderStowing = false;
    }

    @Override
    public void initialize() {
        shoulderStowed = false;
        shoulderStowing = false;
        arm.setWristPosition(WristConstants.kWristStowedAngle);
    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(-10) && !shoulderStowing){
            arm.setShoulderPosition(ShoulderConstants.kShoulderStowedAngle);
            shoulderStowing = true;
        }

        if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderStowedAngle) && shoulderStowing){
            arm.setShoulderPosition(ShoulderConstants.kShoulderFloorCubeAngle);
            shoulderStowed = true;
        }

        if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorCubeAngle) && shoulderStowed){
            arm.setWristPosition(WristConstants.kWristFloorCubeAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
        shoulderStowed = false;
        shoulderStowing = false;
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorCubeAngle) && arm.isWristAtAngle(WristConstants.kWristFloorCubeAngle);
    }


}

