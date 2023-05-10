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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Drivetrain;

public class Logger {
    private static Logger logger;
    private DataLog log = DataLogManager.getLog();
    private Drivetrain drivetrain;
    private DriverOI driverOI;
    private OperatorOI operatorOI;
    private Pose2d position;

    private BooleanLogEntry booleanLog1;
    private DoubleLogEntry doubleLog1, doubleLog2, doubleLog3, doubleLog4, doubleLog5;
    private StringLogEntry stringLog1;
    private DataLogEntry dataLog1;
    private DoubleArrayLogEntry doubleArrayLog1;

    public Logger(){
        doubleLog1 = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");
        doubleLog2 = new DoubleLogEntry(log, "/Drivetrain/Wheel Speed");
        doubleArrayLog1 = new DoubleArrayLogEntry(log, "/Field/Position");
        drivetrain = Drivetrain.getInstance();
        driverOI = DriverOI.getInstance();
        operatorOI = OperatorOI.getInstance();
    }

    public void updateLogs(){
        //Field Pose
        double[] pose = {position.getX(),position.getY(),drivetrain.getHeading()}; 
        doubleArrayLog1.append(pose);
        position = drivetrain.getPose();

        //Drivetrain
        doubleLog1.append(drivetrain.getHeading());
        doubleLog2.append(drivetrain.getSpeed());

        //OI
    }

    public static Logger getInstance() {
        if (logger == null) {
          logger = new Logger();
        }
        return logger;
      }
}

