package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class IntakeFloorCone extends CommandBase{
    private Blinkin blinkin;
    private Claw claw;
    private double initialTime, currentTime;
    private boolean hasCone;

    public IntakeFloorCone(){
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();
        addRequirements(claw);

        initialTime = 0.0;
        currentTime = 0.0;

    }

    @Override
    public void initialize() {
        blinkin.intakingCone();
        claw.setState(ClawState.INTAKING_CONE);     
        hasCone = false;
        claw.intakeCone();
        blinkin.intakingCone();
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

        if(claw.isBothSensors() && !hasCone){
            hasCone = true;
            initialTime = Timer.getFPGATimestamp();
        }
        else if(!claw.isBothSensors()){
            hasCone = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.classifyGamepiece(false);
    }

    @Override
    public boolean isFinished() {
        return hasCone && (currentTime - initialTime) > 0.3;
    }


    
}
