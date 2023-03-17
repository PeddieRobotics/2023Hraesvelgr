// package frc.robot.commands.ArmCommands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Shoulder;
// import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.Arm.ArmState;
// import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;

// public class SetExtendedFloorCubePose extends CommandBase{
//     private Arm arm;
//     private Shoulder shoulder;
//     private Wrist wrist;

//     public SetExtendedFloorCubePose() {
//         arm = Arm.getInstance();
//         addRequirements(arm);

//         shoulder = Shoulder.getInstance();
//         wrist = Wrist.getInstance();
//     }

//     @Override
//     public void initialize() {
//         arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.REGULAR);
//         if(arm.isWristAboveAngle(0) || arm.isShoulderAboveAngle(shoulder.getkExtendedFloorCubeAngle())){
//             arm.setWristPosition(wrist.getkExtendedFloorCubeAngle());
//         }

//         arm.setState(ArmState.FLOOR_INTAKE_CUBE_EXTENDED);
//         arm.setGoalPose(ArmState.NONE);
//     }

//     @Override
//     public void execute() {
//         if(arm.isShoulderAboveAngle(-45)){
//             arm.setWristPosition(wrist.getkExtendedFloorCubeAngle());
//         }

//     }

//     @Override
//     public void end(boolean interrupted){
//         if(!interrupted){
//             arm.holdShoulderPosition();
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return arm.isShoulderAtAngle(shoulder.getkExtendedFloorCubeAngle()) && arm.isWristAtAngle(wrist.getkExtendedFloorCubeAngle());
//     }


// }

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
            wrist.setPosition(20);
        }
        else{
            approachFromAbove = false;
            arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.REGULAR);
            arm.setWristPosition(20);
        }

        arm.setState(ArmState.FLOOR_INTAKE_CONE_EXTENDED);
        arm.setGoalPose(ArmState.NONE);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAtAngle(shoulder.getkTransitoryAngle()) || overshotTargetAngle){
            arm.setWristPosition(wrist.getkExtendedFloorCubeAngle());
            arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.SLOW);
        }

        if(approachFromAbove && arm.isShoulderAtAngle(shoulder.getkTransitoryAngle())){
            overshotTargetAngle = true;
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