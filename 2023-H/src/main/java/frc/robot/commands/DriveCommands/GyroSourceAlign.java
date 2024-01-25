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

public class GyroSourceAlign extends Command {
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
    public GyroSourceAlign() {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();
        
        angleThreshold = 1.0;
        // if(DriverStation.getAlliance().get() == Alliance.Red){
        //    targetAngle = -90; //RED SIDE 
        // }
        // else{
        //     targetAngle = 90;//BLUE SIDE 
        // }
    
        targetAngle = -90; 
        SmartDashboard.putNumber("target angle", 90); 

        //

        error = 0.0;
        thetaController = new PIDController(0.05, 0.0001, 0);
        thetaController.enableContinuousInput(-180, 180);
        FF = 0.2;
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
        Translation2d position;
        // targetAngle = SmartDashboard.getNumber("target angle", 90); 
        double throttle = oi.getSwerveTranslation().getX();
        
        // if (oi.getDriverDPadInput() != DPadDirection.NONE) {
        //     position = oi.getCardinalDirection();
        // } else {
        //     position = oi.getSwerveTranslation();
        //     SmartDashboard.putNumber("field relative input forward axis", position.getX());
        //     SmartDashboard.putNumber("field relative input strafe axis", position.getY());
        // }

        position = new Translation2d(-throttle, 0.0); 

        
        currentAngle = drivetrain.getHeading();
        error = currentAngle - targetAngle;
        if (error < -angleThreshold) {
            llTurn = thetaController.calculate(currentAngle, targetAngle) + FF;
        } else if (error > angleThreshold) {
            llTurn = thetaController.calculate(currentAngle, targetAngle) - FF;
        }

        else {
            llTurn = 0;
        }
        
        

        drivetrain.drive(position, llTurn, false, new Translation2d(0, 0));
        SmartDashboard.putNumber("Gyro angle error", error);
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