package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.Logger;
import frc.robot.utils.RollingAverage;

public class LimelightFront extends Limelight {
    private static LimelightFront limelightFront;

    private RollingAverage txAverage, tyAverage, taAverage, xAverage, rotationAverage;
    private boolean cube;


    private String limelightName = "limelight-front";

    public LimelightFront() {
        txAverage = new RollingAverage();
        tyAverage = new RollingAverage();
        taAverage = new RollingAverage();
        rotationAverage = new RollingAverage();
        xAverage = new RollingAverage(4,getBotpose().getX());
        setPipeline(7);
        SmartDashboard.putNumber("apriltag align p", 0.05); 
        SmartDashboard.putNumber("apriltag align i", 0.00); 
        SmartDashboard.putNumber("apriltag align d", 0.00);
        SmartDashboard.putNumber("apriltag ff", 0.01);
        SmartDashboard.putNumber("apriltag angle threshold", 1.0); 
        
        SmartDashboard.putNumber("apriltag move p", 0.03); 
        SmartDashboard.putNumber("apriltag move i", 0.00);
        SmartDashboard.putNumber("apriltag move d", 0.0);
        SmartDashboard.putNumber("apriltag move threshold", 1.0); 
        SmartDashboard.putNumber("apriltag move ff", 0.40);
        SmartDashboard.putNumber("Current RX", 0.0);
        SmartDashboard.putBoolean("ISSQUARED", false); 
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
        SmartDashboard.putNumber("megatag heading", getBotposeBlue().getRotation().getDegrees());
        SmartDashboard.putNumber("megatag heading average", getRotationAverage()); 
        SmartDashboard.putNumber("Current RX", getRotationAverage()); 
    }

    public void startAveragingX(){
        xAverage = new RollingAverage(4,getBotpose().getX());
    }

    public double getAveragePoseX() {
        return xAverage.getAverage();
    }

    public Translation2d getBotXY() {
        double[] result;
        if(DriverStation.getAlliance().get() == Alliance.Red){
            result = LimelightHelper.getBotPose_wpiRed(limelightName);
        }
        else{
            result = LimelightHelper.getBotPose_wpiBlue(limelightName);
        }
        
        if (result.length > 0.0) {
            return new Translation2d(result[0], result[1]);
        }
        return new Translation2d(0, 0);
    }

    public Pose2d getBotposeBlue() {
        double[] result = LimelightHelper.getBotPose_wpiBlue(limelightName);
        if (result.length > 0.0) {
            return new Pose2d(new Translation2d(result[0], result[1]), new Rotation2d(Math.toRadians(result[5])));
        }
        return new Pose2d();
    }
    public Pose2d getBotpoesRPose2d() {
        double[] result = LimelightHelper.getBotPose_wpiRed(limelightName);
        if (result.length > 0.0) {
            return new Pose2d(new Translation2d(result[0], result[1]), new Rotation2d(Math.toRadians(result[5])));
        }
        return new Pose2d();
    }

    public Pose2d getBotpose() {
        double[] result;
        if(DriverStation.getAlliance().get() == Alliance.Red){
            result = LimelightHelper.getBotPose_wpiRed(limelightName);
        }
        else{
            result = LimelightHelper.getBotPose_wpiBlue(limelightName);
        }

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

    public double getTargetID(){
        return LimelightHelper.getFiducialID(limelightName);
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

    public double getRotationAverage() {
        return rotationAverage.getAverage();
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
            
            // rotationAverage.add(getBotpose().getRotation().getDegrees());//based on alliance of driverstation, awaiting testing 
            rotationAverage.add(getBotposeBlue().getRotation().getDegrees()); 
            
           
        }
    }

    public void resetRollingAverages(){
        txAverage.clear();
        tyAverage.clear();
        taAverage.clear();
        rotationAverage.clear(); 
    }

    public void setPipeline(int pipelineNum) {
        LimelightHelper.setPipelineIndex(limelightName, pipelineNum);
    }

    public int getPipeline(){
        return (int)LimelightHelper.getCurrentPipelineIndex(limelightName);
    }

    public String getJSONDump() {
        return LimelightHelper.getJSONDump(limelightName);
    }

    public void checkForAprilTagUpdates(SwerveDrivePoseEstimator odometry) {
        int tagsSeen = LimelightHelper.getNumberOfAprilTagsSeen(limelightName);
        if (tagsSeen > 1 && this.getBotpose().relativeTo(odometry.getEstimatedPosition()).getTranslation().getNorm() < 0.5) {
            odometry.addVisionMeasurement(this.getBotpose(), Timer.getFPGATimestamp());
        }
    }

    public void forceAprilTagLocalization(SwerveDrivePoseEstimator odometry){
        if(getTv()){
            odometry.addVisionMeasurement(this.getBotpose(), Timer.getFPGATimestamp());
        }
    }

    // public Translation2d getCurrentAprilTag() { // gets the april tag the limelight is currently seeing
    //     return LimelightHelper.getAprilTagCoordinates((int) getTargetID()); // this isn't the closest, it's just the one we're seeing
    // }

    public Translation2d getAprilTagCoordinates(int tagNumber){
        return LimelightHelper.getAprilTagCoordinates(tagNumber);
    }

    public int getClosestColumn(Translation2d pose, boolean isCube){
        return LimelightHelper.getClosestColumn(pose, isCube);
    }    
}