package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
//import frc.robot.commands.AutoCommands.TranslationTest;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
// import frc.robot.utils.UpdateLogs;

public class Target extends CommandBase {
    private final Limelight limelight;
    private final Drivetrain drivetrain;

    private double ff;
    private double steering_adjust;
    private double error;
    private double average_error;
    private double angle_bound;
    private double initialTime;
    private PIDController limelightPIDController;

    private Translation2d translationalSpeed;

    //private static UpdateLogs updatelogs = UpdateLogs.getInstance();


    public Target() {
        limelight = Limelight.getInstance();
        drivetrain = Drivetrain.getInstance();

        addRequirements(limelight);

        limelightPIDController = limelight.getPIDController();

        angle_bound = LimelightConstants.kLimeLightAngleBound;
    }

    @Override
    public void initialize() {
        // Assume by default that we're not locked on a limelight target. Shouldn't be
        // needed, but placed here as a safety on the logic elsewhere.
        drivetrain.setTargeted(false);
        SmartDashboard.putBoolean("Aiming?", true);
        initialTime = Timer.getFPGATimestamp();

        //updatelogs.logCommand("Target");

    }

    @Override
    public void execute() {
        //updatelogs.logCommand("Target");
        OI oi = OI.getInstance();
        ff = limelight.getFF();

        if (limelight.hasTarget()) {
            error = limelight.getTx();
            average_error = limelight.getTxAverage();
            if (average_error < -angle_bound) {
                steering_adjust = limelightPIDController.calculate(average_error) + ff;
            } else if (average_error > angle_bound) {
                steering_adjust = limelightPIDController.calculate(average_error) - ff;
            } else {
                steering_adjust = 0;
            }
        } else {
            steering_adjust = 0;
        }
        if(oi.getTranslation2d().equals(new Translation2d(0, 0))){
            drivetrain.drive(new Translation2d(0,0), steering_adjust, true, new Translation2d(0, 0));
        } else {
            drivetrain.drive(oi.getTranslation2d(), steering_adjust, true, new Translation2d(0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
        SmartDashboard.putBoolean("Aiming?", false);
        // If we end this command with the LL seeing target AND we weren't interrupted
        // (e.g. trigger release), we are locked to target now
        // Otherwise we must be ending immediately because no target was found for
        // alignment
        if (limelight.hasTarget() && !interrupted) {
            drivetrain.setTargeted(true);
        }
    }

    @Override
    public boolean isFinished() {
        OI oi = OI.getInstance();
        // if((Math.abs(drivetrain.getGyroRate()) > 5.0) && (Timer.getFPGATimestamp()-initialTime < 0.5)){
        //     return false;
        //   }
        return ((Math.abs(limelight.getTxAverage()) < angle_bound) && oi.getTranslation2d().equals(new Translation2d(0, 0)));  
        // return false;

    }
}
