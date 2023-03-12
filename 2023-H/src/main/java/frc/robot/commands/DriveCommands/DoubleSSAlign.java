package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.DriverOI;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.Constants.LimelightConstants;

public class DoubleSSAlign extends CommandBase {
    private final Drivetrain drivetrain;
    private PIDController thetaController, yController;
    private DriverOI oi;
    private double forwardDist;
    private String state;
    private Translation2d destinationXY, odometryXY;
    private String limelightName;
    LimelightFront limelightFront;
    LimelightBack limelightBack;
    private Arm arm;
    private Blinkin blinkin;
    private boolean isCube;

    public DoubleSSAlign() {
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
        limelightFront = LimelightFront.getInstance();
        limelightBack = limelightBack.getInstance();
        thetaController = new PIDController(0.035, 0, 0);
        yController = new PIDController(0.055, 0, 0);
        SmartDashboard.putString("side", "right");
        limelightName = "limelightFront";
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        switch (arm.getState()) {
            case L3_CUBE_INVERTED:
                limelightName = "limelightBack";
                break;
            case L3_CONE_INVERTED:
                limelightName = "limelightBack";
                break;
        }
        oi = DriverOI.getInstance();
        state = "6-3";
        destinationXY = LimelightHelper.getCurrentAprilTagCoordinates(limelightName);

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
        if (!state.equals("6-3")) {
            if (SmartDashboard.getString("side", "right").equals("right")) {
                LimelightHelper.setPipelineIndex(limelightName, 2); // POI right
            } else {
                LimelightHelper.setPipelineIndex(limelightName, 1); // POI left
            }
        } else {
            // we might be able to just set this according to the "default lane"
            //SET THIS BASED ON DEFAULT LANE!!!!
            LimelightHelper.setPipelineIndex(limelightName, 0);
        }
        double txAvg;
        if(limelightName.equals("limelightBack")){
            txAvg = limelightBack.getTxAverage();
        } else {
            txAvg = limelightFront.getTxAverage();
        }
        if (Math.abs(txAvg) > LimelightConstants.kLimeLightTranslationAngleBound) {
            yMove = yController.calculate(txAvg, 0);
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
        double startDist = 3; // specifically only used for the 3-1.5 case
        double stopDist = 1.5;
        double currentDist = Math.abs(odometryXY.getX() - destinationXY.getX());
        // might need to slow this down
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
    }
}
