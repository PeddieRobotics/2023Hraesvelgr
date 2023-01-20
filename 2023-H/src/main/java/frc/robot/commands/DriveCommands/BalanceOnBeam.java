package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceOnBeam extends CommandBase{

    private Drivetrain drivetrain;

    private double error;
    private double currentTheta;
    private double drivePower;

    public BalanceOnBeam(){
        drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        this.currentTheta = drivetrain.getPitch();

        error = DriveConstants.BEAM_BALANCED_GOAL_DEGREES - currentTheta;
        drivePower = -Math.min(DriveConstants.BEAM_BALANCED_DRIVE_kP * error, 1);

        //limit max power 
        if (Math.abs(drivePower) > 0.4) {
        drivePower = Math.copySign(0.4, drivePower);
        }

        drivetrain.drive(drivePower);
    }

    //Called once the command ends or is interrupted
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        return Math.abs(error) < DriveConstants.BEAM_BALANCED_ANGLE_THRESHOLD_DEGREES; //Ends when we are in a specific interval
    }
}
