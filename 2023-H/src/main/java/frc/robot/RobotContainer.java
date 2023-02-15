// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.robot.utils.UpdateLogs;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Shuffleboard;
import frc.robot.utils.OI;

public class RobotContainer {
  private final Drivetrain drivetrain;

  private final Claw claw;
  private final Arm arm;

  private final OI oi;
  private final Shuffleboard shuffleboard;
  private final Autonomous autonomous;
  private final LimelightFront limelightFront;
  private final LimelightBack limelightBack;

  private Command autoCommand;

  // private final Blinkin blinkin;

  public RobotContainer() {
    drivetrain = Drivetrain.getInstance();
    drivetrain.setDefaultCommand(new SwerveDriveCommand());

    claw = Claw.getInstance();
    arm = Arm.getInstance();

    oi = OI.getInstance();
    shuffleboard = Shuffleboard.getInstance();

    autonomous = Autonomous.getInstance();
    limelightFront = LimelightFront.getInstance();
    limelightBack = LimelightBack.getInstance();
    // blinkin = Blinkin.getInstance();
  }

  public Command getAutonomousCommand() {
    drivetrain.resetGyro();
    return autonomous.getAutonomousCommand();
  }

  public void resetGyro(){
    drivetrain.resetGyro();
  }

  public void resetPoseToFaceOtherAlliance() {
    if(DriverStation.getAlliance() == Alliance.Blue){
      drivetrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0))));
    }
    else{
      drivetrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(180))));
    }
  }

  public void setupAngleOffsetFromAuto(double target) {
    drivetrain.setTeleOpAngleOffset(target);
  }

  public double getAngleOffsetFromAuto(){
    return autonomous.getAngleOffsetFromAuto();
  }

  public void setArmMode(IdleMode mode){
    arm.setShoulderMode(mode);
  }

  public void setWristMode(IdleMode mode){
    arm.setWristMode(mode);
  }

  public void testAllSystems(){
    claw.testPeriodic();
    arm.testPeriodic();
    
  }

  // public void startLogging(){
  // logs.startLogging();
  // }

  // public void stopLogging(){
  // logs.stopLogging();;
  // }

}