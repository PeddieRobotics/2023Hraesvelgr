package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetExtendedFloorConePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    private boolean shoulderStowed, shoulderStowing, transitory;
    private boolean allowStowing;

    public SetExtendedFloorConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        shoulderStowed = false;
        shoulderStowing = false;
        transitory = false;
        allowStowing = arm.getAllowStowIntake();

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {

        transitory = false;
        shoulderStowed = false;
        shoulderStowing = false;
        allowStowing = arm.getAllowStowIntake();

        if(!allowStowing || (arm.isShoulderAtAngle(shoulder.getkExtendedFloorConeAngle()) && arm.isWristAtAngle(wrist.getkExtendedFloorConeAngle()))){
            shoulderStowed = true;
        }

        arm.setWristPosition(wrist.getkStowedAngle());
        arm.setState(ArmState.MOVING);

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

        if((arm.isShoulderAtAngle(shoulder.getkStowedAngle()) && shoulderStowing) || !allowStowing){
            arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorConeAngle(), SmartMotionArmSpeed.REGULAR);
            shoulderStowed = true;
        }

        if(arm.isShoulderAtAngle(shoulder.getkExtendedFloorConeAngle()) && shoulderStowed){
            arm.setWristPosition(wrist.getkExtendedFloorConeAngle());
        }
    
    }


    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            arm.setState(ArmState.FLOOR_INTAKE_CONE_EXTENDED);
            arm.holdShoulderPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkExtendedFloorConeAngle()) && arm.isWristAtAngle(wrist.getkExtendedFloorConeAngle());
    }


}
