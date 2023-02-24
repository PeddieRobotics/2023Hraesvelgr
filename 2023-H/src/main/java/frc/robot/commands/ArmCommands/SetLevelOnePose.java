package frc.robot.commands.ArmCommands;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelOnePose extends CommandBase{
    private Arm arm;
    private boolean transitory;

    public SetLevelOnePose() {
        arm = Arm.getInstance();
        transitory = false;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        transitory = false;
        if(arm.isShoulderBelowAngle(-50)){
            arm.setWristPosition(WristConstants.kL1Angle);
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kTransitoryAngle);
            transitory = true;
        }
        else{
            arm.setWristPosition(60);
        }
        arm.setState(ArmState.MOVING);

    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(30) && !transitory){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kTransitoryAngle);
            transitory = true;
        }

        if(transitory && arm.isShoulderBelowAngle(-42)){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kL1Angle);
        }

        if(arm.isShoulderBelowAngle(-55)){
            arm.setWristPosition(WristConstants.kL1Angle);
        }
    }

    @Override
    public void end(boolean interrupted){ 
        transitory = false;
        arm.setState(ArmState.L1);
        arm.holdShoulderPosition();
    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(WristConstants.kL1Angle) && arm.isShoulderAtAngle(ShoulderConstants.kL1Angle);
    }


}
