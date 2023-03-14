package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;

public class SetSingleSSCubePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetSingleSSCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setShoulderPositionSmartMotion(shoulder.getkSingleSSCubeAngle(), SmartMotionArmSpeed.REGULAR);
        arm.setWristPosition(wrist.getkSingleSSCubeAngle());
        arm.setState(ArmState.SINGLE_SS_CUBE);
        arm.setGoalPose(ArmState.NONE);

    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            arm.holdShoulderPosition();
        }

    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkSingleSSCubeAngle()) && arm.isWristAtAngle(wrist.getkSingleSSCubeAngle());
    }


}
