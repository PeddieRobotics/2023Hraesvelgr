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

public class PIDToLocation extends Command {
    private double clamp(double num, double min, double max) {
        if (num < min)
            return min;
        if (num > max)
            return max;
        return num;
    }
    // speed clamping that takes negative speeds into account
    private double speedClamp(double speed, double minSpeed, double maxSpeed) {
        if (speed < 0)
            return clamp(speed, -maxSpeed, -minSpeed);
        else
            return clamp(speed, minSpeed, maxSpeed);
    }

    private Drivetrain drivetrain;
    private Limelight limelightFront;

    private double turnThreshold, turnFF, turnTarget;
    private PIDController turnController;

    private double moveThreshold, moveFF, xTarget, yTarget;
    private PIDController xController, yController; 
    
    private double xTargetInitial, turnTargetInitial;
    
    private double moveMultiplier;

    private double timeLimit;
    private double startTime;

    private boolean end1, end2, end3;

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public PIDToLocation(double x, double y, double theta, double timeLimit, double moveThreshold) {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();
        
        turnController = new PIDController(Constants.LimelightConstants.bpTurnP, Constants.LimelightConstants.bpTurnI, 
            Constants.LimelightConstants.bpTurnD);
        turnController.enableContinuousInput(-180, 180);
        turnTargetInitial = theta;
        
        turnThreshold = Constants.LimelightConstants.bpTurnThresh; 
        turnFF = Constants.LimelightConstants.bpTurnFF; 

        xController = new PIDController(Constants.LimelightConstants.bpMoveP, Constants.LimelightConstants.bpMoveI, 
            Constants.LimelightConstants.bpMoveD); 
        xTargetInitial = x; 

        yController = new PIDController(Constants.LimelightConstants.bpMoveP, Constants.LimelightConstants.bpMoveI, 
            Constants.LimelightConstants.bpMoveD);
        yTarget = y; 

        moveMultiplier = 1.0;

        // this.moveThreshold = Constants.LimelightConstants.bpMoveThresh; 
        this.moveThreshold = moveThreshold;
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
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
       
        turnTarget = isRed ? turnTargetInitial - 180 : turnTargetInitial;

        xTarget = isRed ? 16.542 - xTargetInitial : xTargetInitial;
        moveMultiplier = isRed ? 1.0 : -1.0; 
       
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

        // moveThreshold = SmartDashboard.getNumber("bp move tresh", Constants.LimelightConstants.bpMoveThresh); 
        moveFF = SmartDashboard.getNumber("bp move ff", Constants.LimelightConstants.bpMoveFF);  

        turnController.setIZone(5); 

        end1 = false;
        end2 = false;
        end3 = false;

        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (drivetrain.getIsParkedAuto()) {
            end1 = true;
            end2 = true;
            end3 = true;
            return;
        }

        Translation2d position = new Translation2d(0.0, 0.0); 
        double turnAngle = 0.0, xMove = 0.0, yMove = 0.0; 

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
        else
            end1 = true;

        if (xError < -moveThreshold)
            xMove = xController.calculate(currentTX, xTarget) - moveFF * moveMultiplier; 
        else if (xError > moveThreshold)
            xMove = xController.calculate(currentTX, xTarget) + moveFF * moveMultiplier; 
        else
            end2 = true;

        if (yError < -moveThreshold)
            yMove = yController.calculate(currentTY, yTarget) - moveFF * moveMultiplier; 
        else if (yError > moveThreshold)
            yMove = yController.calculate(currentTY, yTarget) + moveFF * moveMultiplier; 
        else
            end3 = true;
        
        xMove = speedClamp(xMove, 0, 1.5);
        yMove = speedClamp(yMove, 0, 1.5);
        
        // you have to negate the input in blue but not red
        position = new Translation2d(xMove * moveMultiplier, yMove * moveMultiplier); 
        drivetrain.drive(position, turnAngle, true, new Translation2d(0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        // drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        return (end1 && end2 && end3) || Timer.getFPGATimestamp() - startTime >= timeLimit; 
    }
}