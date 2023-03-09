package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.RollingAverage;

public class LimelightFront extends Limelight {
    private static LimelightFront limelightFront;

    private RollingAverage txAverage, tyAverage, taAverage, xAverage;
    private boolean cube,level2;

    private String limelightName = "limelight-front";

    public LimelightFront() {
        txAverage = new RollingAverage();
        tyAverage = new RollingAverage();
        taAverage = new RollingAverage();
        xAverage = new RollingAverage(4,getBotpose().getX());

    }

    public static LimelightFront getInstance() {
        if (limelightFront == null) {
            limelightFront = new LimelightFront();
        }
        return limelightFront;
    }

    @Override
    public void periodic() {
        updateRollingAverages();

    }

    public void startAveragingX(){
        xAverage = new RollingAverage(4,getBotpose().getX());
    }

    public double getAveragePoseX() {
        return xAverage.getAverage();
    }

    public Translation2d getBotXY() {
        double[] result = LimelightHelper.getBotPose_wpiBlue(limelightName);
        if (result.length > 0.0) {
            return new Translation2d(result[0], result[1]);
        }
        return new Translation2d(0, 0);
    }

    public Pose2d getBotpose() {
        double[] result = LimelightHelper.getBotPose_wpiBlue(limelightName);
        if (result.length > 0.0) {
            return new Pose2d(new Translation2d(result[0], result[1]), new Rotation2d(Math.toRadians(result[5])));
        }
        return new Pose2d();
    }

    // Tv is whether the limelight has a valid target
    public boolean getTv() {
        return LimelightHelper.getTV(limelightName);
    }

    // Tx is the Horizontal Offset From Crosshair To Target
    public double getTx() {
        return LimelightHelper.getTX(limelightName);
    }

    // Ty is the Vertical Offset From Crosshair To Target
    public double getTy() {
        return LimelightHelper.getTY(limelightName);
    }

    public double getTa() {
        return LimelightHelper.getTA(limelightName);
    }

    public double getTxAverage() {
        return txAverage.getAverage();
    }

    public double getTyAverage() {
        return tyAverage.getAverage();
    }

    public double getTaAverage() {
        return taAverage.getAverage();
    }

    // Class ID of primary neural detector result or neural classifier result
    public double getNeuralClassID() {
        return LimelightHelper.getNeuralClassID(limelightName);
    }

    public double getDistance() {
        if (!hasTarget()) {
            return 0;
        } else {
            // a1 = LL panning angle
            // a2 = additional angle to target
            // tan(a1 + a2) = h/d
            // d = h/tan(a1+a2)
            return (LimelightConstants.kLimelightHeight) /
                    (Math.tan(Math.toRadians(LimelightConstants.kLimelightPanningAngle + getTy())));
        }
    }

    public int getTagsSeen() {
        return LimelightHelper.getNumberOfAprilTagsSeen(limelightName);
    }

    public boolean getCube(){
        return cube;
    }

    public boolean hasTarget() {
        return getTv();
    }

    public boolean targetIsCone() {
        return hasTarget() && getNeuralClassID() == 2;
    }

    public boolean targetIsCube() {
        return hasTarget() && getNeuralClassID() == 1;
    }

    public void updateRollingAverages() {
        if (hasTarget()) {
            txAverage.add(getTx());
            tyAverage.add(getTy());
            taAverage.add(getTa());
        }
    }

    public void resetRollingAverages(){
        txAverage.clear();
        tyAverage.clear();
        taAverage.clear();
    }

    public void setPipeline(int pipelineNum) {
        LimelightHelper.setPipelineIndex(limelightName, pipelineNum);
    }

    public int getPipeline(){
        return (int)LimelightHelper.getCurrentPipelineIndex(limelightName);
    }

    public int setPipelineType(int col){
        level2 = true;
          if (col==2||col==5||col==8) {
            setPipeline(0);
            cube=true;
            return 0;
          }
          cube=false;
          if(level2){
            setPipeline(4);
            return 4;
          }
          setPipeline(5);
          return 5;
    }

    public String getJSONDump() {
        return LimelightHelper.getJSONDump(limelightName);
    }

    public void checkForAprilTagUpdates(SwerveDrivePoseEstimator odometry) {
        int tagsSeen = LimelightHelper.getNumberOfAprilTagsSeen(limelightName);
        // this.getBotpose().relativeTo(odometry.getEstimatedPosition()).getTranslation().getNorm()<.5;
        if (tagsSeen > 1) {
            odometry.addVisionMeasurement(this.getBotpose(), Timer.getFPGATimestamp());
        }
    }

}