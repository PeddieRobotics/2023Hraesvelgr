package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.RollingAverage;

public class LimelightBack extends SubsystemBase {
  private static LimelightBack limelightBack;
  private LimelightHelper limelightHelper=LimelightHelper.createLimelightHelper("limelight-back");

  private RollingAverage txAverage, tyAverage;
  private SendableChooser<Integer> targetAprilTagID, targetColumnNumber, targetRow;

  public LimelightBack() {
    txAverage = new RollingAverage();
    tyAverage = new RollingAverage();

    targetAprilTagID = new SendableChooser<>();
    targetColumnNumber = new SendableChooser<>();
    targetRow = new SendableChooser<>();

    addDropdownOptions();
  }

  public static LimelightBack getInstance() {
    if (limelightBack == null) {
      limelightBack = new LimelightBack();
    }
    return limelightBack;
  }

  @Override
  public void periodic() {
    updateRollingAverages();
    updateLimelightInfoOnDashboard();
    updateCoordstoDashboard();
    setPipeline((int) SmartDashboard.getNumber("pipeline", 0));
  }

  // Tv is whether the limelight has a valid target
  // 1 is true, 0 is false
  public double getTv() {
    return limelightHelper.getLimelightNTDouble("tv");
  }

  public String getJson(){
    return limelightHelper.getJSONDump();
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

  public double getTID() {
    return limelightHelper.getFiducialID();
  }

  public boolean hasTarget() {
    if (getTv() == 1) {
      return true;
    } else
      return false;
  }

  public void updateRollingAverages() {
    if (hasTarget()) {
      txAverage.add(getTx());
      tyAverage.add(getTy());
    }
  }

  public void updateLimelightInfoOnDashboard() {
    SmartDashboard.putBoolean("LL has target", hasTarget());
    SmartDashboard.putNumber("LL tx avg", getTxAverage());
    SmartDashboard.putNumber("angle off limelight", getTx());
    SmartDashboard.putNumber("MAIN TAG ID", getTID());
    updateCoordstoDashboard();
  }

  public void setPipeline(int pipelineNum) {
    limelightHelper.setPipelineIndex(pipelineNum);
  }

  public double getPipeline() {
    return SmartDashboard.getNumber("pipeline", 0);
  }

  public void updateCoordstoDashboard() {
    if (hasTarget()) {
      double[] result = limelightHelper.getBotpose_wpiBlue(); // don't make this getBotpose()
      SmartDashboard.putNumber("array element count", result.length);
      if (result.length > 0.0) {
        SmartDashboard.putNumber("xCOORD", result[0]); // meters
        SmartDashboard.putNumber("yCOORD", result[1]); // meters
        SmartDashboard.putNumber("zCOORD", result[2]); // meters
      }
    }
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

  private void addDropdownOptions() {
    // april tag id
    for (int i = 1; i < 9; i++) {
      targetAprilTagID.addOption("April Tag " + i, i);
    }
    SmartDashboard.putData(targetAprilTagID);
    //column number 
    for (int i = 1; i <= 9; i++) {
      targetColumnNumber.addOption("Column " + i, i);
    }
    SmartDashboard.putData(targetColumnNumber);
    //row number
    for (int i = 1; i <= 3; i++) {
      targetRow.addOption("Level " + i, i);
    }
    SmartDashboard.putData(targetRow);
  }

  public Translation2d getAprilTagCoordinates(int tagNumber) {
    final double add = 0.37 + .4;
    switch (tagNumber) {
      case 1:
        return new Translation2d(7.2431 - add, -2.93659);
      case 2:
        return new Translation2d(7.2431 - add, -1.26019);
      case 3:
        return new Translation2d(7.2431 - add, 0.41621);
      case 4:
        return new Translation2d(7.90832 - add, 2.74161);
      case 5:
        return new Translation2d(-7.90832 + add, 2.74161);
      case 6:
        return new Translation2d(-7.2431 + add, 0.41621);
      case 7:
        return new Translation2d(-7.2431 + add, -1.26019);
      case 8:
        return new Translation2d(-7.2431 + add, -2.93659);
    }
    return new Translation2d(0,0);
  }

  public Translation2d getDestinationCoord(int column) {
    final double add = 0.37 + .4; //gets you to the edge of l1
    switch (column) {
      case 8: //april tag 6
        return new Translation2d(-7.2431 + add, 0.41621);
      case 5: //april tag 7
        return new Translation2d(-7.2431 + add, -1.26019);
      case 2: //april tag 8
        return new Translation2d(-7.2431 + add, -2.93659);
    }
    return new Translation2d(0,0);
  }

  public int getTargetAprilTagID(){
    return targetAprilTagID.getSelected();
  }

  public Translation2d getTargetAprilTagCoord(){
    return getAprilTagCoordinates(getTargetAprilTagID());
  }

  public int getTargetColumnNumber(){
    return targetColumnNumber.getSelected();
  }

  public int getTargetRow(){
    return targetRow.getSelected();
  }

  public Translation2d getTargetColumnCoord(){
    return getDestinationCoord(getTargetColumnNumber());
  }
}