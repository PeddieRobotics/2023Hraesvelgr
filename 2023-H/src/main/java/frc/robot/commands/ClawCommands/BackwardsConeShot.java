package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;

public class BackwardsConeShot extends CommandBase{
    private Claw claw;
    private double initialTime, currentTime,time;

    private Blinkin blinkin;

    public BackwardsConeShot(double t){
        time=t;
        claw = Claw.getInstance();
        blinkin = Blinkin.getInstance();

        addRequirements(claw);

    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();

        claw.setSpeed(-1);
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
        claw.resetGamepieceAlignmentError();
        claw.setGamepieceOperatorOverride(false);
    }

    @Override
    public boolean isFinished() {
        return currentTime - initialTime > time;
    }
        
}