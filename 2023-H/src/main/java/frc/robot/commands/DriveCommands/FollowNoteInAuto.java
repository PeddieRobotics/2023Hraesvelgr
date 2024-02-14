package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;

public class FollowNoteInAuto extends Command {
    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private DriverOI oi;
    private double angleThreshold;
    private double targetAngle;

    private double FF;
    private double llTurn;
    private PIDController thetaController;

    private double currentAngle;
    private double error;

    private static final double DURATION = Integer.MAX_VALUE;
    private static final double EARLY_END_DURATION = 0.6;
    private static final int DO_NOT_SEE_FRAME_LIMIT = 25;
    
    private double startTime;
    
    private int doNotSeeFrameCount;
    private boolean endNow;
    private boolean hasGamePiece;

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public FollowNoteInAuto() {
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();
        
        angleThreshold = 0.5;

        error = 0.0;
        thetaController = new PIDController(0.05, 0.0001, 0);
        thetaController.enableContinuousInput(-180, 180);
        FF = 0.1;
        targetAngle = 0;
        llTurn = 0;

        SmartDashboard.putBoolean("has game piece", false);

        doNotSeeFrameCount = 0;
        endNow = false;
        hasGamePiece = false;
        
        addRequirements(drivetrain);
   }

    @Override
    public void initialize() {
        doNotSeeFrameCount = 0;
        endNow = false;
        startTime = Timer.getFPGATimestamp();
        oi = DriverOI.getInstance();
        limelightFront.setPipeline(1); 
    }

    @Override
    public void execute() {
        // double throttle = oi.getSwerveTranslation().getX();
        double throttle = 0.6;
        Translation2d position = new Translation2d(throttle, 0.0);

        hasGamePiece = SmartDashboard.getBoolean("has game piece", false);

        llTurn = 0;

        if (limelightFront.hasTarget()) {
            doNotSeeFrameCount = 0;
            currentAngle = limelightFront.getTxAverage();
            error = currentAngle - targetAngle;
            SmartDashboard.putNumber("DATA: Error", currentAngle);
            if (error < -angleThreshold)
                llTurn = thetaController.calculate(currentAngle, targetAngle) + FF;
            else if (error > angleThreshold)
                llTurn = thetaController.calculate(currentAngle, targetAngle) - FF;
        }
        else {
            if (doNotSeeFrameCount >= DO_NOT_SEE_FRAME_LIMIT && Timer.getFPGATimestamp() - startTime <= EARLY_END_DURATION) {
                endNow = true;
                return;
            }
            doNotSeeFrameCount++;
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
        return endNow ||
            Timer.getFPGATimestamp() - startTime >= DURATION ||
            hasGamePiece; // OR has the game piece
    }
}
