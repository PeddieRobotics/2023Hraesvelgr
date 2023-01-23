package frc.robot.commands.DriveCommands;

import frc.robot.Robot;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceOnBeam extends CommandBase{

    private Drivetrain drivetrain;

    private double error;
    private Rotation2d rotation;
    private double currentTheta;
    private double drivePower;

    public BalanceOnBeam(){
        Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        this.rotation = drivetrain.getRotation2d();

        error = DriveConstants.BEAM_BALANCED_GOAL_DEGREES -currentTheta;
        drivePower = -Math.min(DriveConstants.BEAM_BALANCED_DRIVE_kP * error, 1);

        //limit max power 
        if (Math.abs(drivePower) > 0.4) {
        drivePower = Math.copySign(0.4, drivePower);
        }

        //there is supposed to be a drive method from drivertrain
        //here in order to supply power to the drivetrain and execute it, 
        //but I cannot find a way to currently do that, investigate after.
        //We will most likely need rotation and translation 2d in order to accomplish
        //based on the current Drive() we have. 
    }

    //Called once the command ends or is interrupted
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        return Math.abs(error) < DriveConstants.BEAM_BALANCED_ANGLE_THRESHOLD_DEGREES; //Ends when we are in a specific interval
    }
}
