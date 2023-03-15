package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.DriverOI;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.subsystems.Blinkin;

public class SingleSSAlign extends CommandBase {
    private final LimelightFront limelightFront;
    private final Drivetrain drivetrain;
    private PIDController thetaController, xController;
    private DriverOI oi;
    private Arm arm;
    private Blinkin blinkin;
    private Claw claw;
    private int scoreSetpoint;
    private boolean initialHeadingCorrectionComplete, initialTargetNotFound;

    public SingleSSAlign() {
        limelightFront = LimelightFront.getInstance();
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();

        thetaController = new PIDController(0.09, 0.0003, 0);
        thetaController.enableContinuousInput(-180, 180);
        xController = new PIDController(0.06, 0, 0);

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        initialHeadingCorrectionComplete = false;
        initialTargetNotFound = false;

        switch (DriverStation.getAlliance()) {
            case Red:
                scoreSetpoint = -90;
                break;
            case Blue:
                scoreSetpoint = 90;
                break;
            default:
                scoreSetpoint = -90;
        }

        oi = DriverOI.getInstance();

        if (!limelightFront.hasTarget()) {
            blinkin.failure();
        } else{
            blinkin.acquiringTarget();
        }
    }

    @Override
    public void execute() {
        double xMove = 0.0;
        double turn = 0.0;
        double turnFF = 0.2;

        double txAvg = limelightFront.getTxAverage();
        if (!initialHeadingCorrectionComplete && Math
                .abs(Math.abs(drivetrain.getHeading()) - scoreSetpoint) > LimelightConstants.kLimelightHeadingBound) {
            turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);

            drivetrain.drive(new Translation2d(oi.getForward() * 1.0, oi.getStrafe() * 1.0),
                    turn + turnFF * Math.signum(turn), true, new Translation2d(0, 0));
        } else if (Math.abs(txAvg) > LimelightConstants.kLimeLightTranslationAngleBound) {
            // If we still don't see a target after the first heading correction stage is complete, stop.
            // Otherwise, proceed indefinitely.
            if (!initialHeadingCorrectionComplete && !LimelightHelper.getTV("limelight-front")) {
                blinkin.failure();
                initialTargetNotFound = true;
                return;
            }
            initialHeadingCorrectionComplete = true;

           xMove = xController.calculate(txAvg, 0);

            drivetrain.drive(new Translation2d(xMove, 0.05 + oi.getStrafe() * 1.0), 0, true, new Translation2d(0, 0));

        } else {
            drivetrain.drive(new Translation2d(0, 0.05 + oi.getStrafe() * 1.0), 0, true, new Translation2d(0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
        if(claw.hasGamepiece()){
            limelightFront.setPipeline(7);
        }
        blinkin.returnToRobotState();

    }

    @Override
    public boolean isFinished() {
        return initialTargetNotFound;
    }
}
