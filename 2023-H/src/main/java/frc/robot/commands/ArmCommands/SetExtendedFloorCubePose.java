package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;

public class SetExtendedFloorCubePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private boolean approachFromAbove, overshotTargetAngle;

    public SetExtendedFloorCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        overshotTargetAngle = false;

        if(arm.isShoulderAboveAngle(shoulder.getkExtendedFloorCubeAngle())){
            approachFromAbove = true;
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
            arm.setWristPosition(88);
        }
        else{
            approachFromAbove = false;
            arm.setWristPosition(wrist.getkExtendedFloorCubeAngle()+30);
        }

        arm.setState(ArmState.FLOOR_INTAKE_CUBE_EXTENDED);
        arm.setGoalPose(ArmState.NONE);
    }

    @Override
    public void execute() {
        if(overshotTargetAngle){
            arm.setWristPosition(wrist.getkExtendedFloorCubeAngle());
            arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.SLOW);
        }

        if(approachFromAbove && arm.isShoulderBelowAngle(shoulder.getkTransitoryAngle())){
            overshotTargetAngle = true;
        }

        if(!approachFromAbove && arm.isWristGreaterThanAngle(wrist.getkExtendedFloorCubeAngle()+5)){
            arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.REGULAR);
        }

        if(!approachFromAbove && arm.isShoulderAboveAngle(-50)){
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
        if(!approachFromAbove){
            return arm.isShoulderAtAngle(shoulder.getkExtendedFloorCubeAngle()) && arm.isWristAtAngle(wrist.getkExtendedFloorCubeAngle());
        }
        else if(approachFromAbove && overshotTargetAngle){
            return arm.isShoulderAtAngle(shoulder.getkExtendedFloorCubeAngle()) && arm.isWristAtAngle(wrist.getkExtendedFloorCubeAngle());
        }
        else{
            return false;
        }
    }


}