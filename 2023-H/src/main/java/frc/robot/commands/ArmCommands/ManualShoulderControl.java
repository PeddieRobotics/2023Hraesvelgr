package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.OperatorOI;

public class ManualShoulderControl extends CommandBase{
    private Arm arm;
    private OperatorOI oi;
    private double currentShoulder, shoulderOffset;

    public ManualShoulderControl() {
        arm = Arm.getInstance();
        // addRequirements(arm);
    }

    @Override
    public void initialize() {
        currentShoulder = arm.getShoulderPosition();
        oi = OperatorOI.getInstance();
    }
    
    @Override
    public void execute() {
        shoulderOffset = oi.getShoulderPIDOffset();
        arm.setShoulderPosition(currentShoulder += shoulderOffset);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
