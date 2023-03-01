package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelThreeCubePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLevelThreeCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(30);
        arm.setShoulderPositionSmartMotion(shoulder.getkL3CubeForwardAngle(), SmartMotionArmSpeed.REGULAR);
        arm.setState(ArmState.MOVING);
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
            arm.setState(ArmState.L3_CUBE_FORWARD);
            arm.holdShoulderPosition();
        }

    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkL3CubeForwardAngle()) && arm.isWristAtAngle(wrist.getkL3CubeForwardAngle());
    }


}
