package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.DriverOI;

public class SetExtendedFloorConePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    private DriverOI oi;

    public SetExtendedFloorConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorConeAngle(), SmartMotionArmSpeed.REGULAR);
        if(arm.isWristLessThanAngle(78) || arm.isShoulderAboveAngle(shoulder.getkExtendedFloorConeAngle())){
            arm.setWristPosition(wrist.getkExtendedFloorConeAngle());
        }

        arm.setState(ArmState.FLOOR_INTAKE_CONE_EXTENDED);
        arm.setGoalPose(ArmState.NONE);

        oi = DriverOI.getInstance();
    }

    @Override
    public void execute() {
        if(oi.touchpadHeld()){
            arm.setWristPosition(wrist.getkExtendedFloorConeAngle()+10);
        }
        else if(arm.isShoulderAboveAngle(-45)){
            arm.setWristPosition(wrist.getkExtendedFloorConeAngle());
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
        return false;
        // return arm.isShoulderAtAngle(shoulder.getkExtendedFloorConeAngle()) && arm.isWristAtAngle(wrist.getkExtendedFloorConeAngle());
    }


}

