package frc.robot.commands.DriveCommands;

import java.sql.Driver;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.DriverOI;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.DriverOI.DPadDirection;

public class ApriltagBotPoseAlign extends Command {
    private Drivetrain drivetrain;
    private Limelight limelightFront;
    private DriverOI oi;

    private double turnThreshold, turnTarget, turnFF;
    private PIDController turnController;

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public ApriltagBotPoseAlign() {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();

        turnTarget = 0;
        turnController = new PIDController(0, 0, 0);
        turnController.enableContinuousInput(-180, 180);

        // if(DriverStation.getAlliance().get() == Alliance.Red){
        //    targetAngle = -60; //RED SIDE 
        // }
        // else{
        //     targetAngle = -120;//BLUE SIDE 
        // }
        //

        addRequirements(drivetrain);

        SmartDashboard.putNumber("turn p", 0.03);
        SmartDashboard.putNumber("turn i", 0.0);
        SmartDashboard.putNumber("turn d", 0.0);
        SmartDashboard.putNumber("turn thresh", 1.0);
        SmartDashboard.putNumber("turn ff", 0.2);
   }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        limelightFront.setPipeline(3); 

        turnController.setP(SmartDashboard.getNumber("turn p", 0.03)); 
        turnController.setI(SmartDashboard.getNumber("turn i", 0.0));
        turnController.setD(SmartDashboard.getNumber("turn d", 0.0));
        turnThreshold = SmartDashboard.getNumber("turn thresh", 1.0);
        turnFF = SmartDashboard.getNumber("turn ff", 0.2);
    }

    @Override
    public void execute() {
        // Translation2d position;
        // double throttle = oi.getSwerveTranslation().getX();
        
        // if (oi.getDriverDPadInput() != DPadDirection.NONE) {
        //     position = oi.getCardinalDirection();
        // } else {
        //     position = oi.getSwerveTranslation();
        //     SmartDashboard.putNumber("field relative input forward axis", position.getX());
        //     SmartDashboard.putNumber("field relative input strafe axis", position.getY());
        // }

        // position = new Translation2d(-throttle, 0.0); 

        Translation2d position = new Translation2d(0.0, 0.0); 
        double turnAngle = 0.0;

        if (limelightFront.hasTarget()) {
            double currentAngle = limelightFront.getRotationAverage();
            double turnError = currentAngle - turnTarget;
            if (turnError < -turnThreshold)
                turnAngle = turnController.calculate(currentAngle, turnTarget) + turnFF;
            else if (turnError > turnThreshold)
                turnAngle = turnController.calculate(currentAngle, turnTarget) - turnFF;
        }

        drivetrain.drive(position, turnAngle, false, new Translation2d(0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(limelightFront.getRotationAverage() - turnTarget) < turnThreshold;
    }
}