package frc.robot.commands.DriveCommands;

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
import frc.robot.utils.Logger;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.DriverOI.DPadDirection;

public class SpeakerSquareThenAlign extends Command {
    private Drivetrain drivetrain;
    private Limelight limelightFront;
    private DriverOI oi;

    private double turnThreshold, turnTarget, turnFF;
    private PIDController turnController;

    private double moveThreshold, moveTarget, moveFF;
    private PIDController moveController;
    private boolean isSquared; 

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public SpeakerSquareThenAlign() {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();

        turnThreshold = 1.0;
        turnTarget = 3;
        turnFF = 0.0;
        turnController = new PIDController(0.05, 0.0, 0);
        turnController.enableContinuousInput(-180, 180);

        moveThreshold = 1.0;
        moveTarget = 0;
        moveFF = 0.4;
        moveController = new PIDController(0.03, 0.0, 0.0);
        
        isSquared = false; 
        SmartDashboard.putBoolean("is squared", isSquared);

        SmartDashboard.putNumber("ll turn p", 0.05);
        SmartDashboard.putNumber("ll turn i", 0.0);
        SmartDashboard.putNumber("ll turn d", 0.0);
        SmartDashboard.putNumber("ll turn thresh", 2000);
        SmartDashboard.putNumber("ll turn ff", 0);

        SmartDashboard.putNumber("ll move p", 0.1);
        SmartDashboard.putNumber("ll move i", 0.0001);
        SmartDashboard.putNumber("ll move d", 0.0);
        SmartDashboard.putNumber("ll move thresh", 0.01);
        SmartDashboard.putNumber("ll move ff", 0);

        addRequirements(drivetrain);
        
   }

    @Override
    public void initialize() {
        isSquared = false;
        oi = DriverOI.getInstance();
        limelightFront.setPipeline(3); 

        turnController.setP(SmartDashboard.getNumber("ll turn p", 0.05)); 
        turnController.setI(SmartDashboard.getNumber("ll turn i", 0.0));
        turnController.setD(SmartDashboard.getNumber("ll turn d", 0.0));
        turnThreshold = SmartDashboard.getNumber("ll turn thresh", 4.0);
        turnFF = SmartDashboard.getNumber("ll turn ff", 0.2);

        moveController.setP(SmartDashboard.getNumber("ll move p", 0.03)); 
        moveController.setI(SmartDashboard.getNumber("ll move i", 0.0));
        moveController.setD(SmartDashboard.getNumber("ll move d", 0.0));
        moveThreshold = SmartDashboard.getNumber("ll move thresh", 1.0); 
        moveFF = SmartDashboard.getNumber("ll move ff", 0.4);
    }

    @Override
    public void execute() {
        double turnAngle = 0;
        SmartDashboard.putBoolean("is squared", isSquared);

        // double throttle = oi.getSwerveTranslation().getX();
        // Translation2d position = new Translation2d(-throttle, 0.0);
        Translation2d translation = new Translation2d(0.0, 0.0); 

        if (limelightFront.hasTarget()) {
            double currentAngle = limelightFront.getRotationAverage();
            double turnError = currentAngle - turnTarget;

            SmartDashboard.putNumber("turn error", turnError);    
                    
            if (turnError < -turnThreshold)
                turnAngle = turnController.calculate(currentAngle, turnTarget) + turnFF;
            else if (turnError > turnThreshold)
                turnAngle = turnController.calculate(currentAngle, turnTarget) - turnFF;
            else
                isSquared = true;

            if (isSquared) {
                double currentTx = limelightFront.getTxAverage(); 
                double moveError = currentTx - moveTarget;
                SmartDashboard.putNumber("move error", moveError);    

                if (moveError > moveThreshold)
                    translation = new Translation2d(0.0, moveController.calculate(currentTx, moveTarget) - moveFF); 
                else if (moveError < -moveThreshold)
                    translation = new Translation2d(0.0, moveController.calculate(currentTx, moveTarget) + moveFF);
            }
        }

        SmartDashboard.putNumber("translation y", translation.getY());
        SmartDashboard.putNumber("turn angle", turnAngle);

        drivetrain.drive(translation, turnAngle, false, new Translation2d(0, 0));
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