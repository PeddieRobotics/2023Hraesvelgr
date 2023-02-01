package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.LimelightConstants;

public class LimelightBack extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static LimelightBack limelight;
  private PIDController limelightPIDController;
  private double ff;

  private RollingAverage txAverage, tyAverage;
  private SendableChooser<Integer> targetAprilTagID, targetColumnNumber, targetRow;

  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
  private NetworkTableEntry tx = limelightTable.getEntry("tx");
  private NetworkTableEntry ty = limelightTable.getEntry("ty");
  private NetworkTableEntry thor = limelightTable.getEntry("thor");
  private NetworkTableEntry tvert = limelightTable.getEntry("tvert");
  private NetworkTableEntry tshort = limelightTable.getEntry("tshort");
  private NetworkTableEntry tlong = limelightTable.getEntry("tlong");
  private NetworkTableEntry ta = limelightTable.getEntry("ta");
  private NetworkTableEntry tv = limelightTable.getEntry("tv");
  private NetworkTableEntry pipeline = limelightTable.getEntry("pipeline");
  private NetworkTableEntry botpose = limelightTable.getEntry("botpose_wpiblue");
  private NetworkTableEntry mainID = limelightTable.getEntry("tid");
  private NetworkTableEntry json = limelightTable.getEntry("json");

  public LimelightBack() {
    limelightPIDController = new PIDController(LimelightConstants.kLimelightP, LimelightConstants.kLimelightI, LimelightConstants.kLimelightD);
    ff = LimelightConstants.kLimelightFF;
    txAverage = new RollingAverage();
    tyAverage = new RollingAverage();

    targetAprilTagID = new SendableChooser<>();
    targetColumnNumber = new SendableChooser<>();
    targetRow = new SendableChooser<>();

    addDropdownOptions();
  }

  public static LimelightBack getInstance() {
    if (limelight == null) {
      limelight = new LimelightBack();
    }
    return limelight;
  }

  @Override
  public void periodic() {
    updateRollingAverages();
    updateLimelightInfoOnDashboard();
    updateCoordstoDashboard();
    setPipeline(SmartDashboard.getNumber("pipeline", 0));
  }

  public PIDController getPIDController() {
    return limelightPIDController;
  }

  public double getFF() {
    return ff;
  }

  // Tv is whether the limelight has a valid target
  // 1 is true, 0 is false
  public double getTv() {
    return tv.getDouble(0.0);
  }

  public String getJson(){
    return json.getString("");
  }

  // Tvert is the vertical sidelength of the rough bounding box (0 - 320 pixels)
  public double getTvert() {
    return tvert.getDouble(0.0);
  }

  // Thor is the horizontal sidelength of the rough bounding box (0 - 320 pixels)
  public double getThor() {
    return thor.getDouble(0.0);
  }

  public double getTshort() {
    return tshort.getDouble(0.0);
  }

  public double getTlong() {
    return tlong.getDouble(0.0);
  }
  
  // Tx is the Horizontal Offset From Crosshair To Target
  public double getTx() {
    return tx.getDouble(0.0);
  }

  // Ty is the Vertical Offset From Crosshair To Target
  public double getTy() {
    return ty.getDouble(0.0);
  }

  public double getTa() {
    return ta.getDouble(0.0);
  }

  public double getTxAverage() {
    return txAverage.getAverage();
  }

  public double getTyAverage() {
    return tyAverage.getAverage();
  }

  public double getTID() {
    return mainID.getDouble(0);
  }

  public boolean hasTarget() {
    if (tv.getDouble(0.0) == 1) {
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

  public void putSmartDashboardOverrides() {
    SmartDashboard.putNumber("LL P", LimelightConstants.kLimelightP);
    SmartDashboard.putNumber("LL I", LimelightConstants.kLimelightI);
    SmartDashboard.putNumber("LL D", LimelightConstants.kLimelightD);
    SmartDashboard.putNumber("LL FF", LimelightConstants.kLimelightFF);
    SmartDashboard.putNumber("LL ANGLE BOUND", LimelightConstants.kLimeLightAngleBound);
  }

  public void setFF(double feedforward) {

    ff = feedforward;
  }

  public void setPipeline(double pipelineNum) {
    pipeline.setNumber(pipelineNum);
  }

  public double getPipeline() {
    return SmartDashboard.getNumber("pipeline", 0);
  }

  public void updateCoordstoDashboard() {
    double[] temp = { 0, 0, 0, 0, 0, 0 };
    if (hasTarget()) {
      double[] result = botpose.getDoubleArray(temp);
      SmartDashboard.putNumber("array element count", result.length);
      if (result.length > 0.0) {
        SmartDashboard.putNumber("xCOORD", result[0]); // meters
        SmartDashboard.putNumber("yCOORD", result[1]); // meters
        SmartDashboard.putNumber("zCOORD", result[2]); // meters
      }
    }
  }
  
  public Translation2d getBotXY() {
    double[] temp = { 0, 0, 0, 0, 0, 0 };
    if (hasTarget()) {
      double[] result = botpose.getDoubleArray(temp);
      if (result.length > 0.0) {
        return new Translation2d(result[0], result[1]);
      }
    }
    return new Translation2d(0, 0);
  }

  public Pose2d getBotpose() {
    double[] temp = { 0, 0, 0, 0, 0, 0 };
    if (hasTarget()) {
      double[] result = botpose.getDoubleArray(temp);
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
