package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetFloorConePose extends CommandBase{
    private Arm arm;
    private boolean stowed;

    public SetFloorConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        stowed = false;
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
            stowed = true;
            arm.setShoulderPosition(ShoulderConstants.kShoulderFloorConeAngle);
        }

        if(stowed && arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorConeAngle)){
            arm.setWristPosition(WristConstants.kWristFloorConeAngle);
        }
    
    }


    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorConeAngle) && arm.isWristAtAngle(WristConstants.kWristFloorConeAngle);
    }


}
