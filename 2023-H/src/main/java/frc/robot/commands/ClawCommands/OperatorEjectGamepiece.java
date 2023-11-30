package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.Constants.ClawConstants;

public class OperatorEjectGamepiece extends CommandBase{
    private Claw claw;
    private OperatorOI operatorOI;
    private Blinkin blinkin;

    public OperatorEjectGamepiece(){
        claw = Claw.getInstance();
        blinkin = Blinkin.getInstance();

        addRequirements(claw);

    }

    @Override
    public void initialize() {
        operatorOI = OperatorOI.getInstance();
        blinkin.specialOperatorFunctionality();
    }

    @Override
    public void execute() {
        if(operatorOI.dPadDownHeld()){
            claw.setSpeed(ClawConstants.kOperatorEjectSpeed);  
        }
        else{
            claw.setSpeed(ClawConstants.kOperatorFastEjectSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
        blinkin.returnToRobotState();

    }

    @Override
    public boolean isFinished() {
        return false;
    }
        
}