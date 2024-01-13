package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.RawLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Wrist;

public class Logger {
  private static Logger logger;
  private DataLog log = DataLogManager.getLog();
  private Superstructure superstructure;
  private Drivetrain drivetrain;
  private DriverOI driverOI;
  private OperatorOI operatorOI;
  private Pose2d fieldPosition;
  private Shoulder shoulder;
  private Wrist wrist;
  private Claw claw;

  private BooleanLogEntry booleanLog1, booleanLog2;
  private DoubleLogEntry gyroAngleEntry, wheelSpeedEntry, shoulderAngleEntry, wristAngleEntry, clawSpeedEntry,
   clawCurrentEntry, shoulderCurrentEntry, wristCurrentEntry;
  private StringLogEntry stringLog1, eventsEntry, robotStateEntry;
  private DataLogEntry dataLog1;
  private DoubleArrayLogEntry fieldPositionEntry;
  private double lastTeleopEnable;

  public Logger(){
    //Setup Subsystems
    drivetrain = Drivetrain.getInstance();
    driverOI = DriverOI.getInstance();
    operatorOI = OperatorOI.getInstance();
    shoulder = Shoulder.getInstance();
    wrist = Wrist.getInstance();
    claw = Claw.getInstance();
    superstructure = Superstructure.getInstance();


    //Double Logs
    gyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");
    wheelSpeedEntry = new DoubleLogEntry(log, "/Drivetrain/Wheel Speed");
    shoulderAngleEntry = new DoubleLogEntry(log, "/Shoulder/Shoulder Angle");
    wristAngleEntry = new DoubleLogEntry(log, "/Wrist/Wrist Angle");
    clawSpeedEntry = new DoubleLogEntry(log, "/Claw/Claw Speed");
    clawCurrentEntry = new DoubleLogEntry(log, "/Claw/Claw Current");
    shoulderCurrentEntry = new DoubleLogEntry(log, "/Shoulder/Shoulder Current");
    wristCurrentEntry = new DoubleLogEntry(log, "/Wrist/Wrist Current");
    fieldPosition = drivetrain.getPose();
    fieldPositionEntry = new DoubleArrayLogEntry(log, "/Field/Position");
    eventsEntry = new StringLogEntry(log, "/Events");
    robotStateEntry = new StringLogEntry(log, "/Superstructure/RobotStates");

    //Boolean Logs

  }

  public void logEvent(String event){
    eventsEntry.append(event);
  }

  public void signalRobotEnable(){
    lastTeleopEnable = Timer.getFPGATimestamp();
  }

  public void updateLogs(){
    //Field Pose
    fieldPosition = drivetrain.getPose();
    double[] pose = {fieldPosition.getX(),fieldPosition.getY(),drivetrain.getHeading()}; 
    fieldPositionEntry.append(pose);

    //Drivetrain
    gyroAngleEntry.append(drivetrain.getHeading());
    wheelSpeedEntry.append(drivetrain.getSpeed());

    //OI

    //Shoulder
    shoulderAngleEntry.append(shoulder.getPosition());
    shoulderCurrentEntry.append(shoulder.getOutputCurrent());

    //Wrist
    wristAngleEntry.append(wrist.getPosition());
    wristCurrentEntry.append(wrist.getOutputCurrent());

    //Claw
    clawSpeedEntry.append(claw.getClawSpeed());
    clawCurrentEntry.append(claw.getOutputCurrent());

    //log robot state state
    robotStateEntry.append((superstructure.getRobotState()!=null||!superstructure.getRobotState().equals("") ? superstructure.getRobotState() : "BAD"));

    //checkCommands.append()
    //logCommand("hi", true);
  }

  public static Logger getInstance() {
    if (logger == null) {
      logger = new Logger();
    }
    return logger;
  }
}

