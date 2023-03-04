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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Shuffleboard.ShuffleboardMain;
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.subsystems.Drivetrain;
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

    @Override
    public void robotInit() {
        // LiveWindow.setEnabled(false);

        robotContainer = new RobotContainer();

        shuffleboard = ShuffleboardMain.getInstance();
        if(OIConstants.kUseDebugModeLayout){
            shuffleboard.setupDebugMode();
            shuffleboard.setupAutoSelector();
        }
        else{
            shuffleboard.setupCompetitionMode();
            shuffleboard.setupAutoSelector();
        }

        PathPlannerServer.startServer(5985); //SHOULD BE 5985!!!!! 5895 WILL NOT WORK!!!!!
        SmartDashboard.putData(CommandScheduler.getInstance());

        ranAutonomousRoutine = false;

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
    }

    @Override
    public void disabledInit() {
        robotContainer.setArmMode(IdleMode.kCoast);
        robotContainer.setWristMode(IdleMode.kCoast);
        // robotContainer.stopLogging();
    }

    @Override
    public void disabledPeriodic() {
    }

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
            robotContainer.setFlipped(true);

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

        // Default pose for the robot to begin teleop is stowed.
        // CommandScheduler.getInstance().schedule(new SetStowedPose());
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        robotContainer.resetGyro();
        robotContainer.setArmMode(IdleMode.kBrake);
        robotContainer.setWristMode(IdleMode.kBrake);

        shuffleboard = ShuffleboardMain.getInstance();
        if(OIConstants.kUseDebugModeLayout){
            shuffleboard.setupDebugMode();
        }
        else{
            shuffleboard.setupCompetitionMode();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }

}