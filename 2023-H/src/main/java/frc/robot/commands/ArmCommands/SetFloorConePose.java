package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetFloorConePose extends CommandBase{
    private Arm arm;
    private boolean shoulderStowed, shoulderStowing;

    public SetFloorConePose() {
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
            // arm.setShoulderPosition(ShoulderConstants.kShoulderFloorConeAngle);
            arm.setShoulderPosition(ShoulderConstants.kShoulderExtendedFloorConeAngle);
            shoulderStowed = true;
        }

        // if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorConeAngle) && shoulderStowed){
        if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderExtendedFloorConeAngle) && shoulderStowed){
            arm.setWristPosition(WristConstants.kWristExtendedFloorConeAngle);
        }
    
    }


    @Override
    public void end(boolean interrupted){
        shoulderStowed = false;
        shoulderStowing = false;
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorConeAngle) && arm.isWristAtAngle(WristConstants.kWristFloorConeAngle);
    }


}
