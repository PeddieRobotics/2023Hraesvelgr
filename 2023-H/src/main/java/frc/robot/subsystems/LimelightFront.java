package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.RollingAverage;

public class LimelightFront extends SubsystemBase {

  private static LimelightFront LimelightFront;

  private double ff;

  private RollingAverage txAverage, tyAverage;

  private NetworkTableInstance LLtableInst = NetworkTableInstance.getDefault();
  private NetworkTable limelightTable = LLtableInst.getTable("limelight-front");
  private NetworkTableEntry tx = limelightTable.getEntry("tx");
  private NetworkTableEntry ty = limelightTable.getEntry("ty");
  private NetworkTableEntry thor = limelightTable.getEntry("thor");
  private NetworkTableEntry tvert = limelightTable.getEntry("tvert");
  private NetworkTableEntry tshort = limelightTable.getEntry("tshort");
  private NetworkTableEntry tlong = limelightTable.getEntry("tlong");
  private NetworkTableEntry ta = limelightTable.getEntry("ta");
  private NetworkTableEntry tv = limelightTable.getEntry("tv");
  private NetworkTableEntry pipeline = limelightTable.getEntry("pipeline");
  private NetworkTableEntry botpose = limelightTable.getEntry("botpose");
  
  // Class ID of primary neural detector result or neural classifier result
  private NetworkTableEntry tclass = limelightTable.getEntry("tclass");

  public LimelightFront() {
    txAverage = new RollingAverage();
    tyAverage = new RollingAverage();
  }

  public static LimelightFront getInstance() {
    if (LimelightFront == null) {
      LimelightFront = new LimelightFront();
    }
    return LimelightFront;
  }

  @Override
  public void periodic() {
    updateRollingAverages();
    updateLimelightInfoOnDashboard();
    setPipeline(SmartDashboard.getNumber("pipeline", 0));
    SmartDashboard.putNumber("BOTPOSE!", botpose());
  }

  public double botpose() {
    return botpose.getDouble(0.0);
  }

  public double getFF() {
    return ff;
  }

  // Tv is whether the limelight has a valid target
  // 1 is true, 0 is false
  public double getTv() {
    return tv.getDouble(0.0);
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

  //Class ID of primary neural detector result or neural classifier result
  public double getTclass() {
    return tclass.getDouble(0.0);
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
          (Math.tan(Math.toRadians(LimelightConstants.kLimelightPanningAngle + ty.getDouble(0.0))));
    }
  }

  public boolean hasTarget() {
    if (tv.getDouble(0.0) == 1) {
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

    pipeline.setNumber((int) SmartDashboard.getNumber("pipeline", 0));
    updatePosetoDashboard();

    SmartDashboard.putNumber("tx", getTx());
  }

  public void setPipeline(double pipelineNum) {
    pipeline.setNumber(pipelineNum);
  }

  public double getPipeline() {
    return SmartDashboard.getNumber("pipeline", 0);
  }

  public void updatePosetoDashboard() {
    double[] temp = { 0, 0, 0, 0, 0, 0 };
    if (hasTarget()) {
      double[] result = botpose.getDoubleArray(temp);
      SmartDashboard.putNumber("array element count", result.length);
      if (result.length > 0.0) {
        SmartDashboard.putNumber("xCOORD", result[0]); //meters
        SmartDashboard.putNumber("yCOORD", result[1]); //meters
        SmartDashboard.putNumber("zCOORD", result[2]); //meters
      }
    }
  }
}
