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
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.DriverOI.DPadDirection;

public class FullOdometryAlign extends Command {
    private Drivetrain drivetrain;
    private Limelight limelightFront;
    private DriverOI oi;

    private double turnThreshold, turnFF, turnTarget;
    private PIDController turnController;

    private double moveThreshold, moveFF, xTarget, yTarget;
    private PIDController xController, yController; 

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public FullOdometryAlign() {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();

        
        turnController = new PIDController(Constants.LimelightConstants.bpTurnP, Constants.LimelightConstants.bpTurnI, 
            Constants.LimelightConstants.bpTurnD);
        turnController.enableContinuousInput(-180, 180);
        turnTarget = 0;
        turnThreshold = Constants.LimelightConstants.bpTurnThresh; 
        turnFF = Constants.LimelightConstants.bpTurnFF; 

        xController = new PIDController(Constants.LimelightConstants.bpMoveP, Constants.LimelightConstants.bpMoveI, 
            Constants.LimelightConstants.bpMoveD); 
        xTarget = 14.45; 

        yController = new PIDController(Constants.LimelightConstants.bpMoveP, Constants.LimelightConstants.bpMoveI, 
            Constants.LimelightConstants.bpMoveD);
        yTarget = 5.27; 

        moveThreshold = Constants.LimelightConstants.bpMoveThresh; 
        moveFF = Constants.LimelightConstants.bpMoveFF; 

        addRequirements(drivetrain);

        SmartDashboard.putNumber("bp turn p", Constants.LimelightConstants.bpTurnP);
        SmartDashboard.putNumber("bp turn i", Constants.LimelightConstants.bpTurnI);
        SmartDashboard.putNumber("bp turn d", Constants.LimelightConstants.bpTurnD);
        SmartDashboard.putNumber("bp turn thresh", Constants.LimelightConstants.bpTurnThresh);
        SmartDashboard.putNumber("bp turn ff", Constants.LimelightConstants.bpTurnFF);

        SmartDashboard.putNumber("bp move p", Constants.LimelightConstants.bpMoveP); 
        SmartDashboard.putNumber("bp move i", Constants.LimelightConstants.bpMoveI); 
        SmartDashboard.putNumber("bp move d", Constants.LimelightConstants.bpMoveD); 
        SmartDashboard.putNumber("bp move thresh", Constants.LimelightConstants.bpMoveThresh); 
        SmartDashboard.putNumber("bp move ff", Constants.LimelightConstants.bpMoveFF); 
   }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        limelightFront.setPipeline(3); 

        turnController.setP(SmartDashboard.getNumber("bp turn p", Constants.LimelightConstants.bpTurnP)); 
        turnController.setI(SmartDashboard.getNumber("bp turn i", Constants.LimelightConstants.bpTurnI));
        turnController.setD(SmartDashboard.getNumber("bp turn d", Constants.LimelightConstants.bpTurnD));
        turnThreshold = SmartDashboard.getNumber("bp turn thresh", Constants.LimelightConstants.bpTurnThresh);
        turnFF = SmartDashboard.getNumber("bp turn ff", Constants.LimelightConstants.bpTurnFF);

        xController.setP(SmartDashboard.getNumber("bp move p", Constants.LimelightConstants.bpMoveP));
        xController.setI(SmartDashboard.getNumber("bp move i", Constants.LimelightConstants.bpMoveI));
        xController.setD(SmartDashboard.getNumber("bp move d", Constants.LimelightConstants.bpMoveD));
        
        yController.setP(SmartDashboard.getNumber("bp move p", Constants.LimelightConstants.bpMoveP));
        yController.setI(SmartDashboard.getNumber("bp move i", Constants.LimelightConstants.bpMoveI));
        yController.setD(SmartDashboard.getNumber("bp move d", Constants.LimelightConstants.bpMoveD));

        moveThreshold = SmartDashboard.getNumber("bp move tresh", Constants.LimelightConstants.bpMoveThresh); 
        moveFF = SmartDashboard.getNumber("bp move ff", Constants.LimelightConstants.bpMoveFF);  

        turnController.setIZone(5); 
    }

    @Override
    public void execute() {
        Translation2d position = new Translation2d(0.0, 0.0); 
        double turnAngle = 0.0, xMove = 0.0, yMove = 0.0; 

        // double currentAngle = limelightFront.getRotationAverage();
        // double currentTX = limelightFront.getRXAverage(); 
        // double currentTY = limelightFront.getRYAverage(); 
        double currentAngle = drivetrain.getPose().getRotation().getDegrees(); 
        double currentTX = drivetrain.getPose().getX(); 
        double currentTY = drivetrain.getPose().getY(); 

        double turnError = currentAngle - turnTarget;
        double xError = currentTX - xTarget; 
        double yError = currentTY - yTarget; 

        if (turnError < -turnThreshold)
            turnAngle = turnController.calculate(currentAngle, turnTarget) + turnFF;
        else if (turnError > turnThreshold)
            turnAngle = turnController.calculate(currentAngle, turnTarget) - turnFF;

        if (xError < -moveThreshold)
            xMove = xController.calculate(currentTX, xTarget) + moveFF; 
        else if (xError > moveThreshold)
            xMove = xController.calculate(currentTX, xTarget) - moveFF; 

        if (yError < -moveThreshold)
            yMove = yController.calculate(currentTY, yTarget) + moveFF; 
        else if (yError > moveThreshold)
            yMove = yController.calculate(currentTY, yTarget) - moveFF; 

        // SmartDashboard.putNumber("x move", xMove);
        // SmartDashboard.putNumber("y move", yMove);
        // SmartDashboard.putNumber("tx avg", currentTX);
        // SmartDashboard.putNumber("ty avg", currentTY);
        SmartDashboard.putNumber("odom current x", currentTX); 
        SmartDashboard.putNumber("odom current Y", currentTY); 
        SmartDashboard.putNumber("odom current rotation", currentAngle); 
        SmartDashboard.putNumber("odom x error", xError);
        SmartDashboard.putNumber("odom y error", yError);
        SmartDashboard.putNumber("odom rotation error", turnError); 
        
        // if (xMove < 0)
        //     xMove = Math.max(xMove, -0.5);
        // else
        //     xMove = Math.min(xMove, 0.5);
        
        // if (yMove < 0)
        //     yMove = Math.max(yMove, -0.5);
        // else
        //     yMove = Math.min(yMove, 0.5); 
        
        
        position = new Translation2d(-xMove, -yMove); 
        drivetrain.drive(position, turnAngle, true, new Translation2d(0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(limelightFront.getRotationAverage() - turnTarget) < turnThreshold;
        return false; 
    }
}