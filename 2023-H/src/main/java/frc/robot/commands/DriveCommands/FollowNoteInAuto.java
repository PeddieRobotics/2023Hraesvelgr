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
    private static final double NOT_SAME_NOTE_THRESHOLD = 2.5;
    private static final double SPEED = 1.5;
    
    private double startTime;
    private int doNotSeeFrameCount;
    private boolean endNow;
    private boolean hasGamePiece;

    private double lastTx;

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
        SmartDashboard.putNumber("DATA: Current TX", 0);
        SmartDashboard.putNumber("DATA: Last TX", 0);
        SmartDashboard.putNumber("DATA: Current-Last Difference", 0);

        doNotSeeFrameCount = 0;
        endNow = false;
        hasGamePiece = false;
        
        addRequirements(drivetrain);
   }

    @Override
    public void initialize() {
        doNotSeeFrameCount = 0;
        endNow = false;
        lastTx = Integer.MAX_VALUE;
        startTime = Timer.getFPGATimestamp();
        oi = DriverOI.getInstance();
        limelightFront.setPipeline(1); 
    }

    @Override
    public void execute() {
        // double throttle = oi.getSwerveTranslation().getX();
        Translation2d position = new Translation2d(SPEED, 0.0);

        hasGamePiece = SmartDashboard.getBoolean("has game piece", false);

        llTurn = 0;

        // when the limelight has a target
        if (limelightFront.hasTarget()) {
            // defined as number of consecutive cycles without note (see below)
            doNotSeeFrameCount = 0;
            // current angle
            currentAngle = limelightFront.getTxAverage();
            // the next problem: if we stop seeing a note, but there is a note directly behind it
            // the robot will turn towards that note and everything breaks
            // so we say do not turn if the tx suddenly jumps
            // lastTx is initialized to Integer.MAX_VALUE, so we detect that here (makes the first game piece seen not seen as an extra note so the command won't end)
            // and only do turning if the absolute difference between the last angle and current angle is below the threshold
            if (lastTx == Integer.MAX_VALUE || Math.abs(lastTx - currentAngle) < NOT_SAME_NOTE_THRESHOLD) {
                lastTx = currentAngle;
                error = currentAngle - targetAngle;
                if (error < -angleThreshold)
                    llTurn = thetaController.calculate(currentAngle, targetAngle) + FF;
                else if (error > angleThreshold)
                    llTurn = thetaController.calculate(currentAngle, targetAngle) - FF;
            }
            SmartDashboard.putNumber("DATA: Current TX", currentAngle);
            SmartDashboard.putNumber("DATA: Last TX", lastTx);
            SmartDashboard.putNumber("DATA: Current-Last Difference", Math.abs(currentAngle - lastTx));
        }
        else {
            // limelight is kind of jittery, sometimes it loses a note for a couple of frames even if it's there
            // so we only say there's no note if there has been no visible note for a certain number of cycles

            // if the path ends at a point where there is no note at all, then we do not want to run this command
            // if we see that there is no note (see above), and this current cycle is within the first x milliseconds,
            // then end the command immediately
            if (doNotSeeFrameCount >= DO_NOT_SEE_FRAME_LIMIT && Timer.getFPGATimestamp() - startTime <= EARLY_END_DURATION) {
                endNow = true;
                return;
            }
            doNotSeeFrameCount++;
        }

        drivetrain.drive(position, llTurn, false, new Translation2d(0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        // endNow is a condition we invented earlier, hasGamePiece is whether the intake detects a piece
        // and set a cap on total time so that we do not get stuck in this command
        return endNow ||
            Timer.getFPGATimestamp() - startTime >= DURATION ||
            hasGamePiece; // OR has the game piece
    }
}
