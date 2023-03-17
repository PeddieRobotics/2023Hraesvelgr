package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetTravelOverBridgePoseInAuto extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetTravelOverBridgePoseInAuto() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle()+20, SmartMotionArmSpeed.REGULAR);
        arm.setState(ArmState.L2_CUBE);
        arm.setGoalPose(ArmState.NONE);

    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-20)){
            arm.setWristPosition(wrist.getkExtendedFloorCubeAngle());
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
        return arm.isShoulderAtAngle(shoulder.getkExtendedFloorCubeAngle()+20) && arm.isWristAtAngle(wrist.getkExtendedFloorCubeAngle());
    }


}
