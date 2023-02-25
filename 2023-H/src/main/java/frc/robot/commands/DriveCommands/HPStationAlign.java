package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.LimelightConstants;

public class HPStationAlign extends CommandBase{
    //private final Limelight limelight;
    private final Drivetrain drivetrain;
    private PIDController thetaController, yController;
    private DriverOI oi;
    private double forwardDist;
    private String state;
    private Translation2d destinationXY, odometryXY;

    public HPStationAlign() { 
        // thetaController = new PIDController(kPThetaController, 0, 0);
        
        // limelight = Limelight.getInstance();

        drivetrain = Drivetrain.getInstance();
        thetaController = new PIDController(0.035, 0, 0);
        yController = new PIDController(0.055, 0, 0);
        SmartDashboard.putString("side", "right");
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        state = "6-3";
        destinationXY = limelight.getClosestAprilTagCoord();
        SmartDashboard.putNumber("DEST X", destinationXY.getX());
        SmartDashboard.putNumber("DEST Y", destinationXY.getY());
        SmartDashboard.putBoolean("over??", false);
    }

    @Override
    public void execute() {
        odometryXY = drivetrain.getPoseAsTranslation2d();
        forwardDist = Math.abs(odometryXY.getX() - destinationXY.getX());
        double yMove = 0.0;
        double turn = 0.0;
        if (Math.abs(drivetrain.getHeading()) > LimelightConstants.kLimelightHeadingBound) {
            turn = thetaController.calculate(drivetrain.getHeading(), 0);
        }
        if(!state.equals("6-3")){
        if (SmartDashboard.getString("side", "right").equals("right")) {
            limelight.setPipeline(2); // point of interest to the right pipeline
        } else {
            limelight.setPipeline(1); // point of interest to the left pipeline
        }
        } else {
            //we might be able to just set this according to the "default lane"
            limelight.setPipeline(0);
        }
        if (Math.abs(limelight.getTxAverage()) > LimelightConstants.kLimeLightTranslationAngleBound) {
            yMove = yController.calculate(limelight.getTxAverage(), 0);
        }

        switch (state) {
            case "6-3": // 6 <= dist < 3
                // snap of sorts
                if ((forwardDist <= 3)) {
                    state = "3-1.5";
                    break;
                }
                drivetrain.drive(new Translation2d(oi.getForward(), yMove), turn, true, new Translation2d(0, 0));
                break;
            case "3-1.5": // 3 <= dist < 1.5
                // align, slowing down over time
                if ((forwardDist <= 1.5)) {
                    state = "1.5-1";
                    break;
                }
                drivetrain.drive(new Translation2d(LimitedSpeedMultiplier(oi.getForward(), odometryXY), yMove), turn,
                        true, new Translation2d(0, 0)); // change this back to using the
                break;
            case "1.5-1": // 1.5 <= dist < 1
                // 1 m/s to acq pt
                if ((forwardDist <= 1)) {
                    state = "1-0.8";
                    break;
                }
                drivetrain.drive(new Translation2d(0.5, yMove), turn, true, new Translation2d(0, 0));
                break;
            case "1-0.8":
                if (forwardDist <= 0.8) {
                    state = "done";
                    break;
                }
                break;
            case "done":
                drivetrain.stopSwerveModules();
        }

        SmartDashboard.putNumber("moveY", yMove);
        SmartDashboard.putNumber("moveTheta", turn);
        SmartDashboard.putNumber("disttoAT", forwardDist);
        SmartDashboard.putString("zzState", state);
    }

    public double LimitedSpeedMultiplier(double input, Translation2d odometryXY) { 
        double speedLimitMultiplier;
        double startDist = 3; //specifically only used for the 3-1.5 case
        double stopDist = 1.5;
        double currentDist = Math.abs(odometryXY.getX() - destinationXY.getX());
        //might need to slow this down
        if (currentDist <= stopDist) {
            return 1;
        } else if (currentDist <= startDist) {
            speedLimitMultiplier = 2 - stopDist / currentDist;
        } else { 
            speedLimitMultiplier = 1;
        }
        SmartDashboard.putNumber("speed multiplier", speedLimitMultiplier);
        return input * speedLimitMultiplier;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        return false;
        // the check for dist<= 0.8 is jsut an extra thing in case there is any case
        // where that could be true w/o it setting the case as done?
    }
}
