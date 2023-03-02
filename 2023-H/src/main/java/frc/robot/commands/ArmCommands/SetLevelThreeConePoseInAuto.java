package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;

public class SetLevelThreeConePoseInAuto extends CommandBase{
    private Arm arm;
    private Claw claw;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLevelThreeConePoseInAuto() {
        arm = Arm.getInstance();
        claw = Claw.getInstance();

        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setState(ArmState.MOVING);

        if(arm.isShoulderAboveAngle(-45)){
            arm.setShoulderPositionSmartMotion(shoulder.getkL3ConeAngle(), SmartMotionArmSpeed.REGULAR);
        }
        
        arm.setWristPosition(103);
    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(90)){
            arm.setShoulderPositionSmartMotion(shoulder.getkL3ConeAngle(), SmartMotionArmSpeed.REGULAR);
            // arm.setShoulderPosition(139);
        }

        if(arm.isShoulderAboveAngle(100.0)){
            arm.setWristPosition(wrist.getkL3ConeAngle());
            // arm.setWristPosition(27);
        }

        if(arm.isShoulderAboveAngle(153)){
            claw.outtakeCone();
        }
  
    }

    @Override
    public void end(boolean interrupted){
        arm.setState(ArmState.L3_CONE_INVERTED);
        claw.stopClaw();
        arm.setWristPosition(103); // prepare the wrist to return
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkL3ConeAngle()) && arm.isWristAtAngle(wrist.getkL3ConeAngle());
    }


}
