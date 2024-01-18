// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shuffleboard.ShuffleboardMain;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.DriverOI;
import frc.robot.utils.OperatorOI;

public class RobotContainer {
    private final Drivetrain drivetrain;

    private final Claw claw;
    private final Arm arm;
    private final Blinkin blinkin;
    private final Superstructure superstructure;

    private final OperatorOI operatorOI;
    private final DriverOI driverOI;
    private final Autonomous autonomous;
    private final LimelightFront limelightFront;
    private final LimelightBack limelightBack;

    private ShuffleboardMain shuffleboard;

    private Command autoCommand;

    private boolean ranAutonomousRoutine;

    public RobotContainer() {
        /**
         * The order of initialization matters here.
         * DO NOT CHANGE unless you enjoy infinite recursive loops and/or null pointer errors.
         * Also, minimize the amount that you "getInstance()" for various subsystems in each other's constructors.
         * This is one thing that can be tricky with singleton design pattern. Try to be modular and minimize
         * too many interdepencies.
         */
        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new SwerveDriveCommand());

        arm = Arm.getInstance();
        claw = Claw.getInstance();
        superstructure = Superstructure.getInstance();

        blinkin = Blinkin.getInstance();
        autonomous = Autonomous.getInstance();
        limelightFront = LimelightFront.getInstance();
        limelightBack = LimelightBack.getInstance();

        operatorOI = OperatorOI.getInstance();
        driverOI = DriverOI.getInstance();

        shuffleboard = ShuffleboardMain.getInstance();

        ranAutonomousRoutine = false;
    }

    public Command getAutonomousCommand() {
        drivetrain.resetGyro();
        return autonomous.getAutonomousCommand();
    }

    public void resetGyro() {
        drivetrain.resetGyro();
    }

    public void setWristMode(IdleMode mode) {
        arm.setWristMode(mode);
    }

    public void setArmMode(IdleMode mode) {
        arm.setShoulderMode(mode);
    }

    public void setFlipped(boolean bool){
        drivetrain.setFlipped(bool);
    }
    
    public boolean isRanAutonomousRoutine() {
        return ranAutonomousRoutine;
    }

    public void setRanAutonomousRoutine(boolean ranAutonomousRoutine) {
        this.ranAutonomousRoutine = ranAutonomousRoutine;
    }

    public void controlLoop(){
        driverOI.controlLoop();
        operatorOI.controlLoop();
    }

}