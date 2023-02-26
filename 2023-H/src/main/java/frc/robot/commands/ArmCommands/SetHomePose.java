package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetHomePose extends CommandBase{
    private Arm arm;
    private boolean transitory;

    public SetHomePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        transitory = false;
        
    }

    @Override
    public void initialize() {
        arm.setWristPosition(WristConstants.kHomeAngle);
        arm.setState(ArmState.MOVING);
    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(WristConstants.kHomeAngle - 40) && !transitory){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kTransitoryAngle);
            transitory = true;
        }

        if(transitory && arm.isShoulderBelowAngle(-42)){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kHomeAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
        transitory = false;
        arm.setState(ArmState.HOME);
        arm.holdShoulderPosition();

    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(WristConstants.kHomeAngle) && arm.isShoulderAtAngle(ShoulderConstants.kHomeAngle);
    }


}
