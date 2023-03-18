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
    private boolean overshotTargetAngle;

    public SetExtendedFloorCubeInAuto() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        overshotTargetAngle = false;
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
            wrist.setPosition(20);

        arm.setState(ArmState.FLOOR_INTAKE_CONE_EXTENDED);
        arm.setGoalPose(ArmState.NONE);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAtAngle(shoulder.getkTransitoryAngle())){
            overshotTargetAngle = true;
        }

        if(overshotTargetAngle){
            arm.setWristPosition(wrist.getkExtendedFloorCubeAngle()-3);
            arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle() + 1, SmartMotionArmSpeed.SLOW);
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
        if(overshotTargetAngle){
            return arm.isShoulderAtAngle(shoulder.getkExtendedFloorCubeAngle() + 1) && arm.isWristAtAngle(wrist.getkExtendedFloorCubeAngle() -3);
        }
        else{
            return false;
        }
    }


}