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
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Logger;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.DriverOI.DPadDirection;

public class SpeakerSquareThenAlign extends Command {
    private Drivetrain drivetrain;
    private Limelight limelightFront;
    private DriverOI oi;

    private double turnThreshold1, turnThreshold2, turnTarget, turnFF;
    private PIDController turnController;

    private double moveThreshold, moveTarget, moveFF;
    private PIDController moveController;
    private boolean isSquared; 

    private double maxThrottle; 

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    // possible use i-zone
    // 
    public SpeakerSquareThenAlign() {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();

        maxThrottle = Constants.LimelightConstants.sourceMaxThrottle; 
        

        //turn controller 
        turnThreshold1 = Constants.LimelightConstants.sourceTurnThresh1;
        turnThreshold2 = Constants.LimelightConstants.sourceTurnThresh2;
        turnTarget = 2;
        turnFF = Constants.LimelightConstants.sourceTurnFF;
        turnController = new PIDController(Constants.LimelightConstants.sourceTurnP, 
        Constants.LimelightConstants.sourceTurnP, 
        Constants.LimelightConstants.sourceTurnD);

        turnController.enableContinuousInput(-180, 180);


        //move controller 
        moveThreshold = Constants.LimelightConstants.sourceMoveThresh;
        moveTarget = 0;
        moveFF = Constants.LimelightConstants.sourceMoveFF;
        moveController = new PIDController(Constants.LimelightConstants.sourceMoveP,
            Constants.LimelightConstants.sourceMoveI, 
            Constants.LimelightConstants.sourceMoveD);
        
        //is squared 
        isSquared = false; 
        SmartDashboard.putBoolean("is squared", isSquared);

        //turn controller values 
        SmartDashboard.putNumber("ll turn p", Constants.LimelightConstants.sourceTurnP);
        SmartDashboard.putNumber("ll turn i", Constants.LimelightConstants.sourceTurnI);
        SmartDashboard.putNumber("ll turn d", Constants.LimelightConstants.sourceTurnD);
        SmartDashboard.putNumber("ll turn thresh 1", Constants.LimelightConstants.sourceTurnThresh1);
        SmartDashboard.putNumber("ll turn thresh 2", Constants.LimelightConstants.sourceTurnThresh2);
        SmartDashboard.putNumber("ll turn ff", Constants.LimelightConstants.sourceTurnFF);

        //move controller values 
        SmartDashboard.putNumber("ll move p", Constants.LimelightConstants.sourceMoveP);
        SmartDashboard.putNumber("ll move i", Constants.LimelightConstants.sourceMoveI);
        SmartDashboard.putNumber("ll move d", Constants.LimelightConstants.sourceMoveD);
        SmartDashboard.putNumber("ll move thresh", Constants.LimelightConstants.sourceMoveThresh);
        SmartDashboard.putNumber("ll move ff", Constants.LimelightConstants.sourceMoveFF);


        SmartDashboard.putNumber("ll turn iZone", Constants.LimelightConstants.sourceTurnIZone);
        SmartDashboard.putNumber("ll move iZone", Constants.LimelightConstants.sourceMoveIZone); 

        SmartDashboard.putNumber("ll max throttle", Constants.LimelightConstants.sourceMaxThrottle);

        addRequirements(drivetrain);
        
   }

    @Override
    public void initialize() {
        isSquared = false;
        oi = DriverOI.getInstance();
        limelightFront.setPipeline(3); 

        //turn controller 
        turnController.setP(SmartDashboard.getNumber("ll turn p", Constants.LimelightConstants.sourceTurnP)); 
        turnController.setI(SmartDashboard.getNumber("ll turn i", Constants.LimelightConstants.sourceTurnI));
        turnController.setD(SmartDashboard.getNumber("ll turn d", Constants.LimelightConstants.sourceTurnD));
        turnThreshold1 = SmartDashboard.getNumber("ll turn thresh 1", Constants.LimelightConstants.sourceTurnThresh1);
        turnThreshold2 = SmartDashboard.getNumber("ll turn thresh 2", Constants.LimelightConstants.sourceTurnThresh2);
        turnFF = SmartDashboard.getNumber("ll turn ff", Constants.LimelightConstants.sourceTurnFF);

        //move controller
        moveController.setP(SmartDashboard.getNumber("ll move p", Constants.LimelightConstants.sourceMoveP)); 
        moveController.setI(SmartDashboard.getNumber("ll move i", Constants.LimelightConstants.sourceMoveI));
        moveController.setD(SmartDashboard.getNumber("ll move d", Constants.LimelightConstants.sourceMoveD));
        moveThreshold = SmartDashboard.getNumber("ll move thresh", Constants.LimelightConstants.sourceMoveThresh); 
        moveFF = SmartDashboard.getNumber("ll move ff", Constants.LimelightConstants.sourceMoveFF);

        turnController.setIZone(SmartDashboard.getNumber("ll move iZone", Constants.LimelightConstants.sourceTurnIZone)); 
        moveController.setIZone(SmartDashboard.getNumber("ll move iZone", Constants.LimelightConstants.sourceMoveIZone)); 

        maxThrottle = SmartDashboard.getNumber("ll max throttle", Constants.LimelightConstants.sourceMaxThrottle);
    }

    @Override
    public void execute() {
        double turnAngle = 0;
        SmartDashboard.putBoolean("is squared", isSquared);

        double throttle = -oi.getSwerveTranslation().getX();

        throttle = Math.min(throttle, maxThrottle); 
        Translation2d translation = new Translation2d(throttle, 0.0); 

        double turnThreshold = isSquared ? turnThreshold2 : turnThreshold1;

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
                    translation = new Translation2d(throttle, moveController.calculate(currentTx, moveTarget) - moveFF); 
                else if (moveError < -moveThreshold)
                    translation = new Translation2d(throttle, moveController.calculate(currentTx, moveTarget) + moveFF);
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