package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.Drake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

public class AutonAlign extends CommandBase {
    private final Limelight limelight;
    private final Drivetrain drivetrain;
    private final double kMax = 0.8;

    private int column,pipe;

    private boolean stopped;

    private PIDController ySnapController, yAlignController, thetaSnapController, thetaAlignController;

    private double ColY;

    private String state;

    private Pose2d initialPose;
    private double initialRotation;

    public AutonAlign(int col) {
        drivetrain = Drivetrain.getInstance();
        limelight = (Math.abs(drivetrain.getPose().getRotation().getDegrees())<90 ^ drivetrain.getFlipped())?LimelightBack.getInstance():LimelightFront.getInstance();

        thetaSnapController = new PIDController(0.055, 0, 0);
        thetaAlignController = new PIDController(0.055, 0, 0);

        ySnapController = new PIDController(2, 0, 0);
        yAlignController = new PIDController(0.035, 0, 0);

        stopped = false;
        column=col;
        pipe=-1;
        addRequirements(drivetrain);

        SmartDashboard.putNumber("minmove", 0.12);
        SmartDashboard.putNumber("snap theta P", 0.015);
        SmartDashboard.putNumber("align theta P", 0.015);
        SmartDashboard.putNumber("snap y P", 2);
        SmartDashboard.putNumber("align y P", 0.035);

        state = "";
        SmartDashboard.putString("ZZSTATE",state);
    }

    @Override
    public void initialize() {
        thetaSnapController.setP(SmartDashboard.getNumber("snap theta P", 0.015));
        thetaAlignController.setP(SmartDashboard.getNumber("align theta P", 0.015));
        ySnapController.setP(SmartDashboard.getNumber("snap y P", 2));
        yAlignController.setP(SmartDashboard.getNumber("align y P", 0.035));

        SmartDashboard.putBoolean("DRIVING TO TARGET", true);
        stopped = false;
        SmartDashboard.putBoolean("stopped", stopped);
        
        yAlignController.reset();
        ySnapController.reset();
        thetaAlignController.reset();
        thetaSnapController.reset();

        thetaSnapController.enableContinuousInput(-180, 180);
        thetaAlignController.enableContinuousInput(-180, 180);

        initialPose=drivetrain.getPose();
        initialRotation=drivetrain.getHeading();

        pipe=limelight.setPipelineType(column);
        state = "SNAP-ALIGN";

    }

    @Override
    public void execute() {

        double tx = limelight.getTxAverage();
        OI oi = OI.getInstance();

        ColY = LimelightConstants.columnDestinationCoords[column].getY();
        pipe=limelight.setPipelineType(column);

        double alignSpeed = SmartDashboard.getNumber("Auton align speed", 1);
        double minMove = SmartDashboard.getNumber("minmove", 0.12);

        SmartDashboard.putNumber("ColY", ColY);
        SmartDashboard.putString("ZZSTATE",state);
        
        switch(state){
            case "SNAP-ALIGN":
                double yMove = 0.0;
                double thetaMove = 0.0;

                // if (Math.abs(drivetrain.getHeading()) > LimelightConstants.kLimeLightAngleBound) {
                //     thetaMove = thetaSnapController.calculate(drivetrain.getPose().getRotation().getDegrees(), 180);
                // }

                if (Math.abs(drivetrain.getPose().getY() - ColY) > LimelightConstants.kLimeLightTranslationBound) {
                    yMove = ySnapController.calculate(drivetrain.getPose().getY(), ColY);
                }
                else{
                    state = "DRIVE-ALIGN";
                }

                if (Math.abs(yMove) > 3) {
                    yMove = Math.signum(yMove) * 3;
                } else if (Math.abs(yMove) < minMove && yMove != 0) {
                    yMove = Math.signum(yMove) * minMove;
                }

                if(drivetrain.getFlipped()) yMove=-yMove;

                SmartDashboard.putNumber("moveY", yMove);

                drivetrain.drive(new Translation2d(LimitedSpeedMultiplier(alignSpeed,1),yMove), thetaMove, true,
                        new Translation2d(0, 0));

                break;
            case "DRIVE-ALIGN":
                yMove = 0.0;

                if (Math.abs(tx) > LimelightConstants.kLimeLightAngleBound) {
                    yMove = yAlignController.calculate(tx, 0);
                }
                
                if (Math.abs(yMove) > 3) {
                    yMove = Math.signum(yMove) * 3;
                } else if (Math.abs(yMove) < minMove) {
                    yMove = Math.signum(yMove) * minMove;
                }

                if(drivetrain.getFlipped()) yMove=-yMove;

                SmartDashboard.putNumber("moveY", yMove);

                drivetrain.drive(new Translation2d(LimitedSpeedMultiplier(alignSpeed,1),-yMove),
                        thetaAlignController.calculate(drivetrain.getPose().getRotation().getDegrees(), 180), false,
                        new Translation2d(0, 0));
                break;
            case "RETURN-TO-PATH":
                
                drivetrain.drive(drivetrain.getTranslationToPoint(initialPose, 2),
                        thetaAlignController.calculate(drivetrain.getHeading(), initialRotation), false,
                        new Translation2d(0, 0));

                if(drivetrain.getTranslationToPoint(initialPose, 2).getNorm()<.15) stopped=true;
                break;

        }
        //SmartDashboard.putNumber("LL dist", limelight.getDistance());
        SmartDashboard.putBoolean("stopped", stopped);
        SmartDashboard.putString("state", state);

    }

    @Override
    public void end(boolean interrupted) {
        //drivetrain.stop();
        stopped = false;

        SmartDashboard.putBoolean("stopped", stopped);
        SmartDashboard.putBoolean("DRIVING TO TARGET", false);
    }

    @Override
    public boolean isFinished() {
        return stopped;
    }

    public double LimitedSpeedMultiplier(double input, double scalar){
        double speedLimitMultiplier = 1;

        if(limelight.getPipeline()!=pipe) return 0;

        double ta = limelight.getTaAverage();
        double taUpperBound = 1.9;
        //double taLowerBound = 0.5;

        if(limelight.getCube()){
            taUpperBound = 1.7;
            //taLowerBound = 0.5;
        } else {
            taUpperBound = .11; //originally 0.04
            //taLowerBound = 0; //doesn't matter, it isn't being used
        }

        if(ta > taUpperBound){
            //state="RETURN-TO-PATH";
            stopped=true;
            return 0;
        }

        if(ta <= taUpperBound){
            speedLimitMultiplier = 1 - ta/taUpperBound;
        }else{
            speedLimitMultiplier = 1;
        }

        

        // double minDist = 0.15;
        // int currentAprilTagID = (int)limelight.getTID();
        // Translation2d colCoords = limelight.getAprilTagCoordinates(currentAprilTagID);
        // Translation2d robotPos = drivetrain.getPoseAsTranslation2d();
        // double currentDist = robotPos.getDistance(colCoords);
        // double distFromMinDist = currentDist-minDist;
        // SmartDashboard.putNumber("current dist", currentDist);
        // SmartDashboard.putNumber("distFromMinDist", distFromMinDist);

        // if(distFromMinDist < 0 ){
        //     return 0;
        // }
        // if(currentDist <(minDist+dist)){
        //     speedLimitMultiplier = (distFromMinDist+0.3)/dist;
        //     //speedLimitMultiplier = distFromMinDist/(dist*2) + 0.4;
        // } else{
        //     speedLimitMultiplier = 1;
        // }

        return scalar*input*speedLimitMultiplier;
    }
}