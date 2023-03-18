package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;

public class SetExtendedFloorCubeInAuto extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetExtendedFloorCubeInAuto() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setState(ArmState.FLOOR_INTAKE_CONE_EXTENDED);
        arm.setGoalPose(ArmState.NONE);
        arm.setWristPosition(wrist.getkExtendedFloorCubeAngle()-6);
        arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle()-3, SmartMotionArmSpeed.REGULAR);
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
            return arm.isShoulderAtAngle(shoulder.getkExtendedFloorCubeAngle()-3) && arm.isWristAtAngle(wrist.getkExtendedFloorCubeAngle() -6);
    }


}