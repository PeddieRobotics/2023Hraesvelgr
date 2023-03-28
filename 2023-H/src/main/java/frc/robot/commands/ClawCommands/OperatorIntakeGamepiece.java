package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.Constants.ClawConstants;

public class OperatorIntakeGamepiece extends CommandBase{
    private Claw claw;
    private OperatorOI operatorOI;
    private Blinkin blinkin;

    public OperatorIntakeGamepiece(){
        claw = Claw.getInstance();
        blinkin = Blinkin.getInstance();

        addRequirements(claw);

    }

    @Override
    public void initialize() {
        operatorOI = OperatorOI.getInstance();
        claw.setSpeed(ClawConstants.kOperatorIntakeSpeed);

        blinkin.specialOperatorFunctionality();
    }

    @Override
    public void execute() {
        if(operatorOI.dPadDownHeld()){
            claw.setSpeed(ClawConstants.kOperatorIntakeSpeed);
        }
        else{
            claw.setSpeed(ClawConstants.kOperatorFastIntakeSpeed);
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