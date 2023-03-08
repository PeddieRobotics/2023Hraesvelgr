package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLLSeekPose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLLSeekPose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(40);
        arm.setState(ArmState.LL_SEEK);

    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(30)){
            arm.setShoulderPositionSmartMotion(shoulder.getkLLSeekAngle(), SmartMotionArmSpeed.REGULAR);
        }

        if(arm.isShoulderBelowAngle(-55)){
            arm.setWristPosition(wrist.getkLLSeekAngle());
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted){
            arm.setState(ArmState.LL_SEEK);
            arm.holdShoulderPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkLLSeekAngle()) && arm.isWristAtAngle(wrist.getkLLSeekAngle());
    }
}
