package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetExtendedFloorConePose extends CommandBase{
    private Arm arm;
    private boolean shoulderStowed, shoulderStowing, transitory;

    public SetExtendedFloorConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        shoulderStowed = false;
        shoulderStowing = false;
        transitory = false;
        
    }

    @Override
    public void initialize() {

        transitory = false;
        shoulderStowed = false;
        shoulderStowing = false;

        if((arm.isShoulderAtAngle(ShoulderConstants.kExtendedFloorConeAngle) && arm.isWristAtAngle(WristConstants.kExtendedFloorConeAngle))){
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
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kExtendedFloorConeAngle);
            shoulderStowed = true;
        }

        if(arm.isShoulderAtAngle(ShoulderConstants.kExtendedFloorConeAngle) && shoulderStowed){
            arm.setWristPosition(WristConstants.kExtendedFloorConeAngle);
        }
    
    }


    @Override
    public void end(boolean interrupted){
        shoulderStowed = false;
        shoulderStowing = false;
        transitory = false;
        arm.setState(ArmState.FLOOR_INTAKE_CONE_EXTENDED);
        arm.holdShoulderPosition();
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kExtendedFloorConeAngle) && arm.isWristAtAngle(WristConstants.kExtendedFloorConeAngle);
    }


}
