// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax.IdleMode;

// import org.littletonrobotics.junction.LoggedRobot;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
// import org.littletonrobotics.junction.io.ByteLogReceiver;
// import org.littletonrobotics.junction.io.LogSocketServer;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Shuffleboard.Shuffleboard;

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
    private Shuffleboard mShuffleboard;
    private boolean ranAutonomousRoutine;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        mShuffleboard = Shuffleboard.getInstance();

        robotContainer = new RobotContainer();
        PathPlannerServer.startServer(5895);
        SmartDashboard.putData(CommandScheduler.getInstance());

        ranAutonomousRoutine = false;
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
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
        mShuffleboard.update();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        robotContainer.setArmMode(IdleMode.kCoast);
        robotContainer.setWristMode(IdleMode.kCoast);
        // robotContainer.stopLogging();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        robotContainer.resetGyro();

        ranAutonomousRoutine = true;

        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {

        // If we are transitioning from autonomous,
        // load in the correct angle offset compared
        // to the initial pose of the path.
        // (Tells us how the robot was setup).
        if (ranAutonomousRoutine) {
            double angle = robotContainer.getAngleOffsetFromAuto();
            robotContainer.setupAngleOffsetFromAuto(angle);
        } else {
            // Always assume field orientation should be opposite
            // where the gyro is zeroed.
            robotContainer.setupAngleOffsetFromAuto(180);

            // For the sake of calculating odometry correctly,
            // make our initial pose on the field such that
            // our robot faces the other alliance.
            // This is only correct if you actually start
            // the robot facing the other alliance!
            robotContainer.resetPoseToFaceOtherAlliance();
        }

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        robotContainer.resetGyro();
        robotContainer.setArmMode(IdleMode.kBrake);
        robotContainer.setWristMode(IdleMode.kBrake);
        LiveWindow.setEnabled(false); // recommended by WPILib documentation for teams with their own test code
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        robotContainer.testAllSystems();
    }

}