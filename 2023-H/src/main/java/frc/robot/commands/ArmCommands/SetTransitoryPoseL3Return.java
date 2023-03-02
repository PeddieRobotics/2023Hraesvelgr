package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetTransitoryPoseL3Return extends CommandBase{
    private Arm arm;
    private boolean transitory,stopped;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetTransitoryPoseL3Return() {
        arm = Arm.getInstance();
        addRequirements(arm);
        transitory = false;

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        stopped=false;
        if(arm.getState() != ArmState.L3_CONE_INVERTED) stopped=true;
        if(stopped) return;
        arm.setWristPosition(103);
        arm.setState(ArmState.MOVING);
    }

    @Override
    public void execute() {
        if(stopped) return;
        if(arm.isWristAboveAngle(75)){
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
        }

        if(arm.isShoulderBelowAngle(80)){
            arm.setWristPosition(wrist.getkTransitoryAngle());
        }
    }

    @Override
    public void end(boolean interrupted){
        transitory = false;
        arm.setState(ArmState.TRANSITION);
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderBelowAngle(75)||stopped;
    }


}
