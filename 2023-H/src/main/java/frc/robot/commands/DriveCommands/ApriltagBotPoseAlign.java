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
    private double angleThreshold;
    private double targetAngle;

    private double FF;
    private double llTurn;
    private PIDController thetaController;

    private double currentAngle;
    private double error;

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public ApriltagBotPoseAlign() {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();
        
        angleThreshold = 1.0;
        // if(DriverStation.getAlliance().get() == Alliance.Red){
        //    targetAngle = -60; //RED SIDE 
        // }
        // else{
        //     targetAngle = -120;//BLUE SIDE 
        // }
        targetAngle = 1.5; 
        SmartDashboard.putNumber("target angle", targetAngle); 

        //

        error = 0.0;
        // thetaController = new PIDController(0.05, 0.0001, 0);
        thetaController = new PIDController(0.06, 0.0, 0);
        thetaController.enableContinuousInput(-180, 180);
        FF = 0.1;
        llTurn = 0;
        addRequirements(drivetrain);
   }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        limelightFront.setPipeline(3); 
    }

    @Override
    public void execute() {
        // thetaController.setP(0.06); 
        // thetaController.setI(0);
        thetaController.setP(SmartDashboard.getNumber("apriltag align p", 0.08)); 
        thetaController.setI(SmartDashboard.getNumber("apriltag align i", 0.00));
        FF = SmartDashboard.getNumber("apriltag ff", 0.02); 

        Translation2d position;
        double throttle = oi.getSwerveTranslation().getX();
        
        // if (oi.getDriverDPadInput() != DPadDirection.NONE) {
        //     position = oi.getCardinalDirection();
        // } else {
        //     position = oi.getSwerveTranslation();
        //     SmartDashboard.putNumber("field relative input forward axis", position.getX());
        //     SmartDashboard.putNumber("field relative input strafe axis", position.getY());
        // }

        position = new Translation2d(-throttle, 0.0); 

        if (limelightFront.hasTarget()) {
            currentAngle = limelightFront.getRotationAverage();
            error = currentAngle - targetAngle;
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
        SmartDashboard.putNumber("DATA: Rotation from BotPose", currentAngle);
        SmartDashboard.putNumber("DATA: llTurn", llTurn);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        return false;

        // return Math.abs(limelightFront.getRotationAverage() - targetAngle) < angleThreshold;
    }
}