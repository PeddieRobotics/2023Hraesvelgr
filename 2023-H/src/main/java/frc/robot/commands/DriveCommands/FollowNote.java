package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.DriverOI;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.DriverOI.DPadDirection;

public class FollowNote extends Command {
    private Drivetrain drivetrain;
    private Limelight limelightFront;
    private DriverOI oi;
    private double angleThreshold;
    private double targetAngle;

    private double FF;
    private double llTurn;
    private PIDController thetaController;

    private double currentAngle;
    private double error;

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public FollowNote() {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();
        
        angleThreshold = 0.5;

        error = 0.0;
        thetaController = new PIDController(0.05, 0.0001, 0);
        thetaController.enableContinuousInput(-180, 180);
        FF = 0.1;
        targetAngle = 0;
        llTurn = 0;
        
        addRequirements(drivetrain);
   }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        limelightFront.setPipeline(1); 
    }

    @Override
    public void execute() {
        double throttle = oi.getSwerveTranslation().getX();
        Translation2d position = new Translation2d(-throttle, 0.0);

        if (limelightFront.hasTarget()) {
            currentAngle = limelightFront.getTxAverage();
            error = currentAngle - targetAngle;
            SmartDashboard.putNumber("DATA: Error", currentAngle);
            if (error < -angleThreshold) {
                llTurn = thetaController.calculate(currentAngle, targetAngle) + FF;
            } else if (error > angleThreshold) {
                llTurn = thetaController.calculate(currentAngle, targetAngle) - FF;
            }

            else {
                llTurn = 0;
            }
        } else {
            llTurn = 0;
        }

        drivetrain.drive(position, llTurn, false, new Translation2d(0, 0));
        SmartDashboard.putNumber("DATA: Note Tx", currentAngle);
        SmartDashboard.putNumber("DATA: Note Turn", llTurn);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        return false;
        // return Math.abs(limelightFront.getTxAverage() - targetAngle) < angleThreshold;
    }
}