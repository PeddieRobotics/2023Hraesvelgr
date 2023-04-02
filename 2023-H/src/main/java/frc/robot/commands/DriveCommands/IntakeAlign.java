package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.LimelightConstants;

public class IntakeAlign extends CommandBase {
    private final LimelightFront limelightFront;
    private final Drivetrain drivetrain;
    private PIDController thetaController, yController;
    private DriverOI oi;
    private boolean horizAlignComplete;

    public IntakeAlign() {
        limelightFront = LimelightFront.getInstance();
        drivetrain = Drivetrain.getInstance();

        horizAlignComplete = false;

        yController = new PIDController(0.07, 0, 0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        horizAlignComplete = false;

        oi = DriverOI.getInstance();

        thetaController.reset();
        yController.reset();

    }

    @Override
    public void execute() {
        double yMove = 0.0;
        double yFF = 0.05;

        Translation2d swerveTranslation = oi.getSwerveTranslation();
        swerveTranslation = swerveTranslation.times(LimelightConstants.kDriveScaleScoreAlign);

        double txAvg = limelightFront.getTxAverage();

        if (Math.abs(txAvg) > LimelightConstants.kLimeLightTranslationScoringAngleBound) {
            yMove = yController.calculate(txAvg, 0);

            drivetrain.drive(new Translation2d(swerveTranslation.getX(), yMove + yFF * Math.signum(yMove)), oi.getRotation(), true, new Translation2d(0, 0));

        } else {
            horizAlignComplete = true;
            drivetrain.drive(new Translation2d(swerveTranslation.getX(), 0), oi.getRotation(), true, new Translation2d(0, 0));
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}