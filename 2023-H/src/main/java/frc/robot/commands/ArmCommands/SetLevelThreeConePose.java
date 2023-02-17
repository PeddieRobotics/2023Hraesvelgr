package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelThreeConePose extends CommandBase{
    private Arm arm;

    public SetLevelThreeConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if(arm.isShoulderBelowAngle(60) || arm.isShoulderAboveAngle(100)){
            arm.setShoulderPosition(ShoulderConstants.kShoulderLevelThreeConeAngle);
        }
        
        arm.setWristPosition(103);
    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(90)){
            arm.setShoulderPosition(ShoulderConstants.kShoulderLevelThreeConeAngle);
        }

        if(arm.isShoulderAboveAngle(100.0)){
            arm.setWristPosition(WristConstants.kWristLevelThreeConeAngle);
        }
  
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderLevelThreeConeAngle) && arm.isWristAtAngle(WristConstants.kWristLevelThreeConeAngle);
    }


}
