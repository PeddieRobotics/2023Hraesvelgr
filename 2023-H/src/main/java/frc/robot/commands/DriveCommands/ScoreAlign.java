package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.LimelightConstants;

public class ScoreAlign extends CommandBase{
    private final LimelightFront limelightFront;
    private final LimelightBack limelightBack;
    //need to figure out if we're going L2 or L3 before we start using the limelight?
    private final Drivetrain drivetrain;
    private PIDController thetaController, yController;
    private DriverOI oi;
    private double forwardDist;
    private String state;
    private Translation2d destinationXY, odometryXY;
    private double aprilTagNum;
    private int scoreSetpoint = 0;

    public ScoreAlign(){
        limelightBack = LimelightBack.getInstance();
        limelightFront = LimelightFront.getInstance();
        drivetrain = Drivetrain.getInstance();
    }
    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        state = "startAlign";
        aprilTagNum = limelight.getTID();
        destinationXY = limelight.getAprilTagCoordinates((int) aprilTagNum);
        if (SmartDashboard.getString("whichTape", "lower").equals("lower")) {
            scoreSetpoint = 0;
        } else if (SmartDashboard.getString("whichTape", "lower").equals("higher")) {
            scoreSetpoint = 180; 
        }
        SmartDashboard.putNumber("DEST X", destinationXY.getX());
        SmartDashboard.putBoolean("over??", false);

        limelight.setPipeline(0);
    }

    @Override
    public void execute() {
        odometryXY = drivetrain.getPoseAsTranslation2d();
        forwardDist = Math.abs(odometryXY.getX() - destinationXY.getX());
        double yMove = 0.0;
        double turn = 0.0;
        if (Math.abs(drivetrain.getHeading()) > LimelightConstants.kLimelightHeadingBound) {
            turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);
        }

        if (SmartDashboard.getString("side", "left").equals("right")) {
            limelight.setPipeline(2); // point of interest to the right pipeline
        } else if (SmartDashboard.getString("side", "left").equals("left")) {
            limelight.setPipeline(1); // point of interest to the left pipeline
        } else {
            // we might be able to just set this according to the "default lane"
            limelight.setPipeline(0); // normal april tag pipeline
        }

        if (Math.abs(limelight.getTxAverage()) > LimelightConstants.kLimeLightTranslationAngleBound) {
            yMove = yController.calculate(limelight.getTxAverage(), 0);
        }
        SmartDashboard.putNumber("moveY", yMove);
        SmartDashboard.putNumber("moveTheta", turn);
        SmartDashboard.putNumber("disttoEnd", forwardDist);
        SmartDashboard.putString("zzState", state);
        switch (state) {
            case "startAlign":
                if (forwardDist <= SmartDashboard.getNumber("startLL", 0.25)) {
                    state = "startLLAlign";
                    break;
                }
                if (limelight.hasTarget() && forwardDist <= 0.8) {
                    drivetrain.drive(new Translation2d(LimitedSpeedMultiplier(oi.getForward(), odometryXY), yMove),
                            turn, true, new Translation2d(0, 0));
                } else {
                    drivetrain.drive(new Translation2d(oi.getForward(), oi.getStrafe()), turn, true,
                            new Translation2d(0, 0));
                }
                break;

            case "startLLAlign":
                if (forwardDist <= SmartDashboard.getNumber("finalXMax", 0.05)) {
                    state = "fixY";
                } else {
                    if (Math.abs(limelight.getTxAverage()) > LimelightConstants.kLimeLightTranslationAngleBound) {
                        if (SmartDashboard.getString("side", "left").equals("center")) {
                            // april tag, don't do anything
                        } else if (SmartDashboard.getString("whichTape", "lower").equals("lower")) {
                            limelight.setPipeline(LimelightConstants.kLLTapeLowerPipeline);
                        } else if (SmartDashboard.getString("whichTape", "lower").equals("upper")) {
                            limelight.setPipeline(LimelightConstants.kLLTapeUpperPipeline);
                        }
                        yController.setP(SmartDashboard.getNumber("yControllerP", 0.02));
                        yMove = yController.calculate(limelight.getTxAverage(), 0);
                    } else {
                        yMove = 0;
                    }
                }

                drivetrain.drive(new Translation2d(LimitedSpeedMultiplier(oi.getForward(), odometryXY), yMove), turn,
                        true, new Translation2d(0, 0));
                break;
            case "fixY":
                if (Math.abs(limelight.getTxAverage()) <= LimelightConstants.kLimeLightTranslationAngleBound){
                    state = "done";
                }
                drivetrain.drive(new Translation2d(0, yMove), turn, true, new Translation2d(0, 0));
                break;
            case "done":
                drivetrain.stopSwerveModules();
        }
    }

    public double LimitedSpeedMultiplier(double input, Translation2d odometryXY) { 
        double speedLimitMultiplier;
        double startDist = .8; // this is specifically only used for the startAlign case
        double stopDist = SmartDashboard.getNumber("finalXMax", 0.05);
        double currentDist = Math.abs(odometryXY.getX() - destinationXY.getX());

        if (currentDist <= stopDist) {
            return 1;
        } else if (currentDist <= startDist) {
            speedLimitMultiplier = 1 - stopDist / currentDist;
        } else { 
            speedLimitMultiplier = 1;
        }
        return input * (speedLimitMultiplier/2);
    }

    @Override
    public void end(boolean interrupted) {
        oi.setAllowBackward(false);
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
