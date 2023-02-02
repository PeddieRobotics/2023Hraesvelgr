package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetShoulderPosition extends CommandBase{
    private Arm arm;
    private double setpoint;

    public SetShoulderPosition(double setpoint){
        arm = Arm.getInstance();
        this.setpoint = setpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPosition(setpoint);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}