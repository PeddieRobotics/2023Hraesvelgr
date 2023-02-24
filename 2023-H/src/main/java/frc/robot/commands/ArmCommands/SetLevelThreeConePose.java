package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
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
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kL3ConeAngle);
        }
        
        arm.setWristPosition(103);
        arm.setState(ArmState.MOVING);

    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(90)){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kL3ConeAngle);
        }

        if(arm.isShoulderAboveAngle(100.0)){
            arm.setWristPosition(WristConstants.kL3ConeAngle);
        }
  
    }

    @Override
    public void end(boolean interrupted){
        arm.setState(ArmState.L3_CONE_INVERTED);
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kL3ConeAngle) && arm.isWristAtAngle(WristConstants.kL3ConeAngle);
    }


}
