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

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public SpeakerSquareThenAlign() {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();

        turnThreshold = 1.0;
        turnTarget = 1.5;
        turnFF = 0.1;
        turnController = new PIDController(0.06, 0.0, 0);
        turnController.enableContinuousInput(-180, 180);

        moveThreshold = 1.0;
        moveTarget = 0;
        moveFF = 0.4;
        moveController = new PIDController(0.03, 0.0, 0.0);

        addRequirements(drivetrain);
   }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        limelightFront.setPipeline(3); 
    }

    @Override
    public void execute() {
        boolean isSquared = false;

        if (!isSquared) {
            double turnAngle;

            turnController.setP(SmartDashboard.getNumber("apriltag align p", 0.08)); 
            turnController.setI(SmartDashboard.getNumber("apriltag align i", 0.00));
            turnFF = SmartDashboard.getNumber("apriltag ff", 0.02); 

            // double throttle = oi.getSwerveTranslation().getX();
            // Translation2d position = new Translation2d(-throttle, 0.0);
            Translation2d position = new Translation2d(0.0, 0.0); 

            if (limelightFront.hasTarget()) {
                double currentAngle = limelightFront.getRotationAverage();
                double error = currentAngle - turnTarget;
                if (error < -turnThreshold)
                    turnAngle = turnController.calculate(currentAngle, turnTarget) + turnFF;
                else if (error > turnThreshold)
                    turnAngle = turnController.calculate(currentAngle, turnTarget) - turnFF;
                else
                    turnAngle = 0;
            }
            else
                turnAngle = 0;

            drivetrain.drive(position, turnAngle, false, new Translation2d(0, 0));
            return;
        }

        if (limelightFront.hasTarget()) {
            Translation2d translation;
            double currentTx = limelightFront.getTxAverage(); 
            double error = currentTx - moveTarget;
            if (error > moveThreshold)
                translation = new Translation2d(0.0, moveController.calculate(-currentTx, moveTarget) + moveFF); 
            else if (error < -turnThreshold)
                translation = new Translation2d(0.0, moveController.calculate(-currentTx, moveTarget) - moveFF); 
            else
                translation = new Translation2d(0.0, 0.0); 
            drivetrain.drive(translation, 0.0, true, new Translation2d(0, 0));
        }

        // thetaController.setP(0.06); 
        // thetaController.setI(0);
        
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