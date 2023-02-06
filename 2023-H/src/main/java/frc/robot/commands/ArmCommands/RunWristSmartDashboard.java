package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RunWristSmartDashboard extends CommandBase{
    private Arm arm;

    public RunWristSmartDashboard(){
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setWristSpeed(SmartDashboard.getNumber("wrist speed % setpoint", 0.0));
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopWrist();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}