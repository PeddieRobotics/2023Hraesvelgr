package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;

public class SetLevelOnePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    private boolean transitory;

    public SetLevelOnePose() {
        arm = Arm.getInstance();
        transitory = false;
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        transitory = false;
        if(arm.isShoulderBelowAngle(-50)){
            arm.setWristPosition(wrist.getkL1Angle());
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.SLOW);
            transitory = true;
        }
        else{
            arm.setWristPosition(60);
        }
        arm.setState(ArmState.MOVING);

    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(30) && !transitory){
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
            transitory = true;
        }

        if(transitory && arm.isShoulderBelowAngle(-39)){
            arm.setShoulderPositionSmartMotion(shoulder.getkL1Angle(), SmartMotionArmSpeed.SLOW);
        }

        if(arm.isShoulderBelowAngle(-55)){
            arm.setWristPosition(wrist.getkL1Angle());
        }
    }

    @Override
    public void end(boolean interrupted){ 
        if(!interrupted){
            arm.setState(ArmState.L1);
            arm.holdShoulderPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(wrist.getkL1Angle()) && arm.isShoulderAtAngle(shoulder.getkL1Angle());
    }


}
