package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetStowedPose extends CommandBase{
    private Arm arm;
    private boolean transitory;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetStowedPose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        transitory = false;
        
        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        transitory = false;
        arm.setWristPosition(wrist.getkStowedAngle());
        arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.FAST);
        arm.setState(ArmState.MOVING);

    }

    @Override
    public void execute() {
        if(arm.isShoulderBelowAngle(-35)){
            arm.setShoulderPositionSmartMotion(shoulder.getkStowedAngle(), SmartMotionArmSpeed.SLOW);
        }
    }

    @Override
    public void end(boolean interrupted){
        // If the stow results in the shoulder's angle being above the desired angle, pull it in towards the limit sensor
        // if(shoulder.getAngle() > shoulder.getkStowedAngle()){
        //     shoulder.setPercentOutput(-0.1);
        // }
        if(!interrupted){
            arm.setState(ArmState.STOWED);
        }

    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(wrist.getkStowedAngle()) && arm.isShoulderAtAngle(shoulder.getkStowedAngle());
    }


}
