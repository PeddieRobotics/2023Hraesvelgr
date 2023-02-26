package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Claw.ClawState;

public class IntakeCone extends CommandBase{
    private Claw claw;
    private LimelightFront limelightFront;
    private boolean sawCone;

    public IntakeCone(){
        claw = Claw.getInstance();
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        limelightFront = LimelightFront.getInstance();
        limelightFront.setPipeline(7);
        claw.intakeCone();
        sawCone = false;
        claw.setState(ClawState.INTAKING);
    }

    @Override
    public void execute() {
        if(limelightFront.hasTarget()){
            sawCone = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
        limelightFront.setPipeline(0);
    }

    @Override
    public boolean isFinished() {
        if(claw.monitorCurrentForSuccessfulIntake()){
            claw.stopClaw();
            if(sawCone){
                claw.setState(ClawState.CONE);
            }
            else{
                claw.setState(ClawState.CUBE);
            }
            return true;
        }

        return false;
    }

    
}
