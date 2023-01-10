package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
// import frc.robot.commands.AutoCommands.ReversalPointTest;
import frc.robot.utils.RollingAverage;
// import frc.robot.utils.UpdateLogs;

public class Limelight extends SubsystemBase{
    
    private static Limelight limelight;

    private PIDController limelightPIDController;
    private double feedForward;

    private RollingAverage xAverage, yAverage;

    //private static UpdateLogs updatelogs = UpdateLogs.getInstance();

    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = limelightTable.getEntry("tx");
    private NetworkTableEntry ty = limelightTable.getEntry("ty");
    private NetworkTableEntry thor = limelightTable.getEntry("thor");
    private NetworkTableEntry tvert = limelightTable.getEntry("tvert");
    private NetworkTableEntry ta = limelightTable.getEntry("ta");
    private NetworkTableEntry tv = limelightTable.getEntry("tv");

    public Limelight(){
        limelightPIDController = new PIDController(LimelightConstants.kLimelightP, LimelightConstants.kLimelightI, LimelightConstants.kLimelightD);
        feedForward = LimelightConstants.kLimelightFF;
        xAverage = new RollingAverage();
        yAverage = new RollingAverage();
    }
                          
    public static Limelight getInstance(){
        if(limelight == null){
            limelight = new Limelight();
        }
        return limelight;
    }

  @Override
  public void periodic() {
    updateRollingAverages();
    //updatelogs.updateLimelightLogData();
  }

  public PIDController getPIDController(){
    return limelightPIDController;
  }

  public double getFF(){
    return feedForward;
  }

  // Tv is whether the limelight has a valid target
  // 1 is true, 0 is false
  public double getTv(){
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

  // Target Area (0% of image to 100% of image)
  public double getTa(){
    return ta.getDouble(0.0);
  }

  public double getTxAverage(){
    return xAverage.getAverage();
  }

  public double getTyAverage(){
    return yAverage.getAverage();
  }

  public boolean hasTarget(){
    if (tv.getDouble(0.0)==1){
      return true;
    } else
      return false;
  }

  public void updateRollingAverages(){
    if(hasTarget()){
      xAverage.add(getTx());
      yAverage.add(getTy());
    }
  }

  public void updateLimelightInfoOnDashboard(){
    SmartDashboard.putBoolean("LL has target", hasTarget());
    SmartDashboard.putNumber("LL tx avg", getTxAverage());
    SmartDashboard.putNumber("angle off limelight", getTx());

  }
  public void putSmartDashboardOverrides(){
    SmartDashboard.putNumber("LL P", LimelightConstants.kLimelightP);
    
    SmartDashboard.putNumber("LL I", LimelightConstants.kLimelightI);
    SmartDashboard.putNumber("LL D", LimelightConstants.kLimelightD);
    SmartDashboard.putNumber("LL FF", LimelightConstants.kLimelightFF);
    SmartDashboard.putNumber("LL ANGLE BOUND", LimelightConstants.kLimeLightAngleBound);
  }

  public void setFF(double feedforward){
    feedForward = feedforward;
  }

}