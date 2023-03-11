package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetCompactFloorCubePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    private boolean shoulderStowed, shoulderStowing, transitory;

    public SetCompactFloorCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        shoulderStowed = false;
        shoulderStowing = false;
        transitory = false;

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        shoulderStowed = false;
        shoulderStowing = false;
        transitory = false;
        
        if(arm.isShoulderAtAngle(shoulder.getkCompactFloorCubeAngle()) && arm.isWristAtAngle(wrist.getkCompactFloorCubeAngle())){
            shoulderStowed = true;
        }

        arm.setWristPosition(wrist.getkStowedAngle());
        arm.setState(ArmState.FLOOR_INTAKE_CUBE_COMPACT);

    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(30) && !shoulderStowing){
            
            if(!transitory){
                arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
                transitory = true;
            }
            if(transitory && arm.isShoulderBelowAngle(-39)){
                arm.setShoulderPositionSmartMotion(shoulder.getkStowedAngle(), SmartMotionArmSpeed.SLOW);
                shoulderStowing = true;
            }
        }

        if(arm.isShoulderAtAngle(shoulder.getkStowedAngle()) && shoulderStowing){
            arm.setShoulderPositionSmartMotion(shoulder.getkCompactFloorCubeAngle(), SmartMotionArmSpeed.REGULAR);
            shoulderStowed = true;
        }

        if(arm.isShoulderAtAngle(shoulder.getkCompactFloorCubeAngle()) && shoulderStowed){
            arm.setWristPosition(wrist.getkCompactFloorCubeAngle());
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkCompactFloorCubeAngle()) && arm.isWristAtAngle(wrist.getkCompactFloorCubeAngle());
    }


}

