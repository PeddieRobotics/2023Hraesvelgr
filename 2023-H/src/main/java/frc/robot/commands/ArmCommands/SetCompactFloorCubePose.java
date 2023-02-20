package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetCompactFloorCubePose extends CommandBase{
    private Arm arm;
    private boolean shoulderStowed, shoulderStowing;

    public SetCompactFloorCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        shoulderStowed = false;
        shoulderStowing = false;
    }

    @Override
    public void initialize() {
        shoulderStowed = false;
        shoulderStowing = false;
        arm.setWristPosition(WristConstants.kStowedAngle);
    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(-10) && !shoulderStowing){
            arm.setShoulderPosition(ShoulderConstants.kStowedAngle);
            shoulderStowing = true;
        }

        if(arm.isShoulderAtAngle(ShoulderConstants.kStowedAngle) && shoulderStowing){
            arm.setShoulderPosition(ShoulderConstants.kCompactFloorCubeAngle);
            shoulderStowed = true;
        }

        if(arm.isShoulderAtAngle(ShoulderConstants.kCompactFloorCubeAngle) && shoulderStowed){
            arm.setWristPosition(WristConstants.kCompactFloorCubeAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
        shoulderStowed = false;
        shoulderStowing = false;
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kCompactFloorCubeAngle) && arm.isWristAtAngle(WristConstants.kCompactFloorCubeAngle);
    }


}

