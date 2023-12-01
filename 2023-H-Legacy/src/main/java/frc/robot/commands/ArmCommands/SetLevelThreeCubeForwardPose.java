package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;

public class SetLevelThreeCubeForwardPose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLevelThreeCubeForwardPose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(78);
        arm.setShoulderPositionSmartMotion(shoulder.getkL3CubeForwardAngle(), SmartMotionArmSpeed.REGULAR);
        arm.setState(ArmState.L3_CUBE_FORWARD);
        arm.setGoalPose(ArmState.NONE);

    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-20)){
            arm.setWristPosition(wrist.getkL3CubeForwardAngle());
        }
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            arm.holdShoulderPosition();
        }

    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkL3CubeForwardAngle()) && arm.isWristAtAngle(wrist.getkL3CubeForwardAngle());
    }


}
