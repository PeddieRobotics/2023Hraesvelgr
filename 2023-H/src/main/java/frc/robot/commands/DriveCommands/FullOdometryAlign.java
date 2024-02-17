package frc.robot.commands.DriveCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.Constants;

public class FullOdometryAlign extends Command {
    private Drivetrain drivetrain;
    private Limelight limelightFront;

    private double turnThreshold, turnFF, turnTarget;
    private PIDController turnController;

    private double moveThreshold, moveFF, xTarget, yTarget;
    private PIDController xController, yController; 

    private double timeLimit;
    private double startTime;

    private boolean end = false;

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public FullOdometryAlign(double x, double y, double theta, double timeLimit) {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();

        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        
        turnController = new PIDController(Constants.LimelightConstants.bpTurnP, Constants.LimelightConstants.bpTurnI, 
            Constants.LimelightConstants.bpTurnD);
        turnController.enableContinuousInput(-180, 180);
        turnTarget = isRed ? theta + 180 : theta;
        
        turnThreshold = Constants.LimelightConstants.bpTurnThresh; 
        turnFF = Constants.LimelightConstants.bpTurnFF; 

        xController = new PIDController(Constants.LimelightConstants.bpMoveP, Constants.LimelightConstants.bpMoveI, 
            Constants.LimelightConstants.bpMoveD); 
        xTarget = isRed ? 16.542 - x : x; 

        yController = new PIDController(Constants.LimelightConstants.bpMoveP, Constants.LimelightConstants.bpMoveI, 
            Constants.LimelightConstants.bpMoveD);
        // yTarget = isRed ? 8.211 - y : y; 
        yTarget = y; 

        moveThreshold = Constants.LimelightConstants.bpMoveThresh; 
        moveFF = Constants.LimelightConstants.bpMoveFF; 

        this.timeLimit = timeLimit;

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

        startTime = Timer.getFPGATimestamp();
        end = false;
    }

    @Override
    public void execute() {
        Translation2d position = new Translation2d(0.0, 0.0); 
        double turnAngle = 0.0, xMove = 0.0, yMove = 0.0; 

        double currentAngle = drivetrain.getPose().getRotation().getDegrees(); 
        double currentTX = drivetrain.getPose().getX(); 
        double currentTY = drivetrain.getPose().getY(); 

        double turnError = currentAngle - turnTarget;
        double xError = currentTX - xTarget; 
        double yError = currentTY - yTarget; 

        SmartDashboard.putNumber("pid move turn error", turnError);
        SmartDashboard.putNumber("pid move x error", xError);
        SmartDashboard.putNumber("pid move y error", yError);

        boolean end1 = false, end2 = false, end3 = false;

        if (turnError < -turnThreshold)
            turnAngle = turnController.calculate(currentAngle, turnTarget) + turnFF;
        else if (turnError > turnThreshold)
            turnAngle = turnController.calculate(currentAngle, turnTarget) - turnFF;
        else
            end1 = true;

        if (xError < -moveThreshold)
            xMove = xController.calculate(currentTX, xTarget) + moveFF; 
        else if (xError > moveThreshold)
            xMove = xController.calculate(currentTX, xTarget) - moveFF; 
        else
            end2 = true;

        if (yError < -moveThreshold)
            yMove = yController.calculate(currentTY, yTarget) + moveFF; 
        else if (yError > moveThreshold)
            yMove = yController.calculate(currentTY, yTarget) - moveFF; 
        else
            end3 = true;

        end |= end1 && end2 && end3;
        
        if (xMove < 0)
            xMove = Math.max(xMove, -0.5);
        else
            xMove = Math.min(xMove, 0.5);
        
        if (yMove < 0)
            yMove = Math.max(yMove, -0.5);
        else
            yMove = Math.min(yMove, 0.5); 
        
        
        position = new Translation2d(-xMove, -yMove); 
        drivetrain.drive(position, turnAngle, true, new Translation2d(0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        // drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        return end || Timer.getFPGATimestamp() - startTime >= timeLimit; 
    }
}