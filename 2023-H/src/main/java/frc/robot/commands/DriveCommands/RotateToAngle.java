package frc.robot.commands.DriveCommands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.LimelightConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateToAngle extends CommandBase{

    private Drivetrain drivetrain;
    private double initialHeading, currentHeading, targetAngle;

    private PIDController thetaController;

    public RotateToAngle(double targetAngle){
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);

        thetaController = new PIDController(0.05, 0.0001, 0);
        thetaController.enableContinuousInput(-180, 180);

        SmartDashboard.putNumber("RotateToAngle P", 0.05);
        SmartDashboard.putNumber("RotateToAngle I", 0.0001);
        SmartDashboard.putNumber("RotateToAngle D", 0.0);
        SmartDashboard.putNumber("RotateToAngle FF", 0.2);
    }

    @Override
    public void initialize(){
        initialHeading = drivetrain.getHeading();

        thetaController.reset();

        thetaController.setP(SmartDashboard.getNumber("RotateToAngle P", 0.05));
        thetaController.setI(SmartDashboard.getNumber("RotateToAngle I", 0.0001));
        thetaController.setD(SmartDashboard.getNumber("RotateToAngle D", 0.0));
    }

    @Override
    public void execute(){
        double turn = 0.0;
        double turnFF = 0.2;

        currentHeading = drivetrain.getHeading();

        turn = thetaController.calculate(currentHeading, targetAngle);

        drivetrain.drive(new Translation2d(), turn + turnFF * Math.signum(turn), true, new Translation2d(0, 0));

    }

    //Called once the command ends or is interrupted
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        return Math.abs(Math.abs(currentHeading) - targetAngle) < LimelightConstants.kLimelightHeadingBound;
    }
}

