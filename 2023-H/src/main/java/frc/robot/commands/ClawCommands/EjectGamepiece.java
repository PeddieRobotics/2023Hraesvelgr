package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArmCommands.SetTransitoryPoseL3Return;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Claw.ClawState;

public class EjectGamepiece extends CommandBase{
    private Claw claw;
    private Arm arm;
    private double initialTime, currentTime;
    private final double minimumEjectionTime = 0.5; // Always try for 1 second to eject the current game piece

    public EjectGamepiece(){
        claw = Claw.getInstance();
        arm = Arm.getInstance();
        addRequirements(claw);

        initialTime = 0.0;
        currentTime = 0.0;
    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        claw.outtakeCone();

        // if(claw.hasCone()){
        //     claw.outtakeCone();
        // }
        // else if(claw.hasCube()){
        //     claw.outtakeCube();
        // }
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
        claw.setState(ClawState.EMPTY);
    }

    @Override
    public boolean isFinished() {
        return (currentTime-initialTime) > minimumEjectionTime;
    }

    
}
