package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ClawCommands.EjectGamepiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevel3Part1Pose extends CommandBase{
    private Arm arm;
    private Claw claw;

    public SetLevel3Part1Pose() {
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        if(arm.isShoulderAboveAngle(-45)){
            arm.setShoulderPosition(ShoulderConstants.kL3ConePart1Angle);
        }
        
        arm.setWristPosition(103);
    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(90)){
            arm.setShoulderPosition(ShoulderConstants.kL3ConePart1Angle);
            // arm.setShoulderPosition(139);
        }

        if(arm.isShoulderAboveAngle(100.0)){
            arm.setWristPosition(WristConstants.kL3ConeAngle);
            // arm.setWristPosition(27);
        }

        if(arm.isShoulderAboveAngle(153)){
            claw.outtakeCone();
        }
  
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kL3ConePart1Angle) && arm.isWristAtAngle(WristConstants.kL3ConeAngle);
    }


}