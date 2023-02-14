package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.RollingAverage;

public class LimelightFront extends Limelight {
  private static LimelightFront limelightFront;

  private RollingAverage txAverage, tyAverage;

  private LimelightHelper limelightHelper=LimelightHelper.createLimelightHelper("limelight-front");

  public LimelightFront() {
    txAverage = new RollingAverage();
    tyAverage = new RollingAverage();
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
    updateLimelightInfoOnDashboard();
    setPipeline((int) SmartDashboard.getNumber("pipeline", 0));
    SmartDashboard.putNumberArray("BOTPOSE_WPIBLUE!", limelightHelper.getBotpose_wpiBlue());
  }

  public Translation2d getBotXY() {
    if (hasTarget()) {
      double[] result = limelightHelper.getBotpose_wpiBlue();
      if (result.length > 0.0) {
        return new Translation2d(result[0], result[1]);
      }
    }
    return new Translation2d(0, 0);
  }

  public Pose2d getBotpose() {
    if (hasTarget()) {
      double[] result = limelightHelper.getBotpose_wpiBlue();
      if (result.length > 0.0) {
        return new Pose2d(new Translation2d(result[0], result[1]), new Rotation2d(Math.toRadians(result[5])));
      }
    }
    return null;
  }

  // Tv is whether the limelight has a valid target
  // 1 is true, 0 is false
  public double getTv() {
    return limelightHelper.getLimelightNTDouble("tv");
  }

  // Tvert is the vertical sidelength of the rough bounding box (0 - 320 pixels)
  public double getTvert() {
    return limelightHelper.getLimelightNTDouble("tvert");
  }

  // Thor is the horizontal sidelength of the rough bounding box (0 - 320 pixels)
  public double getThor() {
    return limelightHelper.getLimelightNTDouble("thor");
  }

  public double getTshort() {
    return limelightHelper.getLimelightNTDouble("tshort");
  }

  public double getTlong() {
    return limelightHelper.getLimelightNTDouble("tlong");
  }
  
  // Tx is the Horizontal Offset From Crosshair To Target
  public double getTx() {
    return limelightHelper.getTX();
  }

  // Ty is the Vertical Offset From Crosshair To Target
  public double getTy() {
    return limelightHelper.getTY();
  }

  public double getTa() {
    return limelightHelper.getTA();
  }

  public double getTxAverage() {
    return txAverage.getAverage();
  }

  public double getTyAverage() {
    return tyAverage.getAverage();
  }

  //Class ID of primary neural detector result or neural classifier result
  public double getTclass() {
    return limelightHelper.getLimelightNTDouble("tclass");
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

  public boolean hasTarget() {
    if (getTv() == 1) {
      return true;
    } else
      return false;
  }

  public boolean targetIsCone(){
    return hasTarget() && getTclass() == 2;
  }

  public boolean targetIsCube(){
    return hasTarget() && getTclass() == 1;
  }

  public void updateRollingAverages() {
    if (hasTarget()) {
      txAverage.add(getTx());
      tyAverage.add(getTy());
    }
  }

  public void updateLimelightInfoOnDashboard() {
    SmartDashboard.putNumber("LL target distance", getDistance());
    SmartDashboard.putNumber("LL vt error", getTy());
    SmartDashboard.putNumber("LL hz error", getTx());

    // SmartDashboard.putNumber("LL dist", getDistance());
    SmartDashboard.putBoolean("LL has target", hasTarget());
    SmartDashboard.putNumber("LL tx avg", getTxAverage());

    limelightHelper.setPipelineIndex((int) SmartDashboard.getNumber("pipeline", 0));
    updatePosetoDashboard();

    SmartDashboard.putNumber("tx", getTx());
  }

  public void setPipeline(int pipelineNum) {
    limelightHelper.setPipelineIndex(pipelineNum);
  }

  public double getPipeline() {
    return SmartDashboard.getNumber("pipeline", 0);
  }

  public void updatePosetoDashboard() {
    if (hasTarget()) {
      double[] result = limelightHelper.getBotpose_wpiBlue();
      SmartDashboard.putNumber("array element count", result.length);
      if (result.length > 0.0) {
        SmartDashboard.putNumber("xCOORD", result[0]); //meters
        SmartDashboard.putNumber("yCOORD", result[1]); //meters
        SmartDashboard.putNumber("zCOORD", result[2]); //meters
      }
    }
  }

  public String getJSONDump(){
    return limelightHelper.getJSONDump();
  }

}