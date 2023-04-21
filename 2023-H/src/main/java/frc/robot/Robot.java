// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

//import org.littletonrobotics.junction.LoggedRobot;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
// import org.littletonrobotics.junction.io.ByteLogReceiver;
// import org.littletonrobotics.junction.io.LogSocketServer;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Shuffleboard.ShuffleboardMain;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Constants.OIConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private static RobotContainer robotContainer;
    private ShuffleboardMain shuffleboard;
    private boolean ranAutonomousRoutine;
    private PowerDistribution pdh;

    @Override
    public void robotInit() {

        // Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        // if (isReal()) {
        //     Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        //     Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        //     new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        // } else {
        //     setUseTiming(false); // Run as fast as possible
        //     String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        //     Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
        //     Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        // }

        // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        // Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        LiveWindow.setEnabled(false);

        robotContainer = new RobotContainer();

        shuffleboard = ShuffleboardMain.getInstance();
        if(OIConstants.kUseDebugModeLayout){
            // Set up a REV PDH in order to get key status information
            // pdh = new PowerDistribution(1, ModuleType.kRev);

            shuffleboard.setupDebugMode();
            shuffleboard.setupAutoSelector();
        }
        else{
            shuffleboard.setupCompetitionMode();
            shuffleboard.setupAutoSelector();
        }

        // PathPlannerServer.startServer(5985); //SHOULD BE 5985!!!!! 5895 WILL NOT WORK!!!!!
        SmartDashboard.putData(CommandScheduler.getInstance());

        ranAutonomousRoutine = false;

        LimelightFront.getInstance().setPipeline(3);
        LimelightBack.getInstance().setPipeline(0);
    }

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Update all tabs on shuffleboard, reading/writing as applicable to each field.
        // This controls both keeping track of key information, as well as updating key parameters
        // from Shuffleboard.
        shuffleboard.update();

        // if(OIConstants.kUseDebugModeLayout){
        //     double current8 = pdh.getCurrent(8);
        //     double current9 = pdh.getCurrent(9);
        //     double current14 = pdh.getCurrent(14);
        //     SmartDashboard.putNumber("Current Channel 14", current14);
        // }

    }

    @Override
    public void disabledInit() {
        robotContainer.setArmMode(IdleMode.kCoast);
        robotContainer.setWristMode(IdleMode.kCoast);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        robotContainer.resetGyro();

        Claw.getInstance().classifyGamepiece();

        ranAutonomousRoutine = true;

        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        robotContainer.setRanAutonomousRoutine(ranAutonomousRoutine);
        Blinkin.getInstance().returnToRobotState();
        if(Claw.getInstance().getState() == ClawState.CUBE){
            Claw.getInstance().setSpeed(ClawConstants.kCubeHoldSpeed);
        }
        //Claw.getInstance().classifyGamepiece();

        if (!ranAutonomousRoutine) {
            robotContainer.setFlipped(true);
            robotContainer.resetPoseToFaceOtherAlliance();
        }

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        // Make sure pipelines are set correctly to the defaults
        // LimelightFront.getInstance().setPipeline(7);
        // LimelightBack.getInstance().setPipeline(0);

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        robotContainer.resetGyro();
        robotContainer.setArmMode(IdleMode.kBrake);
        robotContainer.setWristMode(IdleMode.kBrake);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }

}