package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;

public class SetExtendedFloorCubePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private boolean approachFromAbove, overshotTargetAngle;

    private double wristTargetAngle;

    public SetExtendedFloorCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();

        wristTargetAngle = 0;
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
            if(arm.getState() == ArmState.HOME || arm.getState() == ArmState.STOWED){
                shoulder.setPercentOutput(0);
            }
            approachFromAbove = false;
            wristTargetAngle = 185;
            arm.setWristPosition(wristTargetAngle);
        }

        arm.setState(ArmState.FLOOR_INTAKE_CUBE_EXTENDED);
        arm.setGoalPose(ArmState.NONE);
    }

    @Override
    public void execute() {
        if(approachFromAbove){
            if(overshotTargetAngle){
                wristTargetAngle = wrist.getkExtendedFloorCubeAngle();
                arm.setWristPosition(wristTargetAngle);
                arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.SLOW);
            }

            if(arm.isShoulderBelowAngle(shoulder.getkTransitoryAngle())){
                overshotTargetAngle = true;
            }
        }

        if(!approachFromAbove){
            if(arm.isShoulderAboveAngle(-65)){
                wristTargetAngle = wrist.getkExtendedFloorCubeAngle()+15;
                arm.setWristPosition(wristTargetAngle);
            }

            if(arm.isShoulderAboveAngle(-55)){
                wristTargetAngle = wrist.getkExtendedFloorCubeAngle()+5;
                arm.setWristPosition(wristTargetAngle);
                arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.REGULAR);
            }

            if(arm.isShoulderAboveAngle(-45)){
                wristTargetAngle = wrist.getkExtendedFloorCubeAngle();
                arm.setWristPosition(wristTargetAngle);
                arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.REGULAR);
            }

            // If the wrist has met or exceeded its goal threshold, ensure the shoulder moves to its goal position.
            // This way, the pose will always finish.
            if(arm.isWristAtAngle(wristTargetAngle) || arm.isWristGreaterThanAngle(wristTargetAngle)){
                wristTargetAngle = wrist.getkExtendedFloorCubeAngle();
                arm.setWristPosition(wristTargetAngle);
                arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.REGULAR);
            }
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