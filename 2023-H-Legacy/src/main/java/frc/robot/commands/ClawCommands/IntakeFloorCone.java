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

    private boolean blinkinSuccess;

    public IntakeFloorCone(){
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();
        addRequirements(claw);

        initialTime = 0.0;
        currentTime = 0.0;

        blinkinSuccess = false;
    }

    @Override
    public void initialize() {
        blinkin.intakingCone();
        claw.setState(ClawState.INTAKING_CONE);     
        hasCone = false;
        claw.intakeCone();
        blinkinSuccess = false;
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

        if(claw.isBackSensor() && !hasCone){
            hasCone = true;
            initialTime = Timer.getFPGATimestamp();
            if(!blinkinSuccess){
                Blinkin.getInstance().success();
                blinkinSuccess = true;
            }
        }
        else if(!claw.isBackSensor()){
            hasCone = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.classifyGamepiece();
    }

    @Override
    public boolean isFinished() {
        return hasCone && (currentTime - initialTime) > 0.3;
    }


    
}
