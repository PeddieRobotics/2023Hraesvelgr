package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetHomePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private boolean transitory;

    public SetHomePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        transitory = false;

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        
    }

    @Override
    public void initialize() {
        arm.setWristPosition(wrist.getkHomeAngle());
        arm.setState(ArmState.MOVING);
    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(wrist.getkHomeAngle() - 40) && !transitory){
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
            transitory = true;
        }

        if(transitory && arm.isShoulderBelowAngle(-39)){
            arm.setShoulderPositionSmartMotion(shoulder.getkHomeAngle(), SmartMotionArmSpeed.SLOW);
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
        return arm.isWristAtAngle(wrist.getkHomeAngle()) && arm.isShoulderAtAngle(shoulder.getkHomeAngle());
    }


}
