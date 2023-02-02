package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.LimelightConstants;

public class LimelightFront extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static LimelightFront LimelightFront;
  private PIDController limelightFrontPIDController;
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

  public LimelightFront() {
    limelightFrontPIDController = new PIDController(LimelightConstants.kLimelightP, LimelightConstants.kLimelightI, LimelightConstants.kLimelightD);
    ff = LimelightConstants.kLimelightFF;
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
    updateSecondLimelightFromDashboard();
  }

  public double botpose() {
    return botpose.getDouble(0.0);
  }

  public PIDController getPIDController() {
    return limelightFrontPIDController;
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

  public double getDistance() {
    if (ty.getDouble(0.0) == 0) {
      return 0;
    } else {
      double targHeight = SmartDashboard.getNumber("TARGET HEIGHT", 112);
      return (targHeight - LimelightConstants.kLimelightHeight) /
          (Math.tan(Math.toRadians(LimelightConstants.kLimelightAngle + LimelightConstants.kLimelightPanningAngle + ty.getDouble(0.0))));
    }
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
    SmartDashboard.putNumber("LL DISTANCE!!", getDistance() - 10);
    SmartDashboard.putNumber("LL vt error", getTy());
    SmartDashboard.putNumber("LL hz error", getTx());

    // SmartDashboard.putNumber("LL dist", getDistance());
    SmartDashboard.putBoolean("LL has target", hasTarget());
    SmartDashboard.putNumber("LL tx avg", getTxAverage());

    pipeline.setNumber((int) SmartDashboard.getNumber("pipeline", 0));
    updatePosetoDashboard();

    SmartDashboard.putNumber("tx", getTx());
  }

  public void updateSecondLimelightFromDashboard() {
    limelightFrontPIDController.setP(SmartDashboard.getNumber("LL P", LimelightConstants.kLimelightP));
    limelightFrontPIDController.setI(SmartDashboard.getNumber("LL I", LimelightConstants.kLimelightI));
    limelightFrontPIDController.setD(SmartDashboard.getNumber("LL D", LimelightConstants.kLimelightD));
    LimelightFront.setFF(SmartDashboard.getNumber("LL FF", LimelightConstants.kLimelightFF));
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
