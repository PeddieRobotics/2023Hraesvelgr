package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetCompactFloorConePose extends CommandBase{
    private Arm arm;
    private boolean shoulderStowed, shoulderStowing, transitory;

    public SetCompactFloorConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        shoulderStowed = false;
        shoulderStowing = false;
        transitory = false;
        
    }

    @Override
    public void initialize() {
        shoulderStowed = false;
        shoulderStowing = false;
        transitory = false;

        if(arm.isShoulderAtAngle(ShoulderConstants.kCompactFloorConeAngle) && arm.isWristAtAngle(WristConstants.kCompactFloorCubeAngle)){
            shoulderStowed = true;
        }

        arm.setWristPosition(WristConstants.kStowedAngle);
        arm.setState(ArmState.MOVING);

    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(30) && !shoulderStowing){
            
            if(!transitory){
                arm.setShoulderPositionSmartMotion(ShoulderConstants.kTransitoryAngle);
                transitory = true;
            }
            if(transitory && arm.isShoulderBelowAngle(-42)){
                arm.setShoulderPositionSmartMotion(ShoulderConstants.kStowedAngle);
                shoulderStowing = true;
            }
        }

        if(arm.isShoulderAtAngle(ShoulderConstants.kStowedAngle) && shoulderStowing){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kCompactFloorConeAngle);
            shoulderStowed = true;
        }

        if(arm.isShoulderAtAngle(ShoulderConstants.kCompactFloorConeAngle) && shoulderStowed){
            arm.setWristPosition(WristConstants.kCompactFloorConeAngle);
        }
    }


    @Override
    public void end(boolean interrupted){
        shoulderStowed = false;
        shoulderStowing = false;
        transitory = false;
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kCompactFloorConeAngle) && arm.isWristAtAngle(WristConstants.kCompactFloorConeAngle);
    }


}
