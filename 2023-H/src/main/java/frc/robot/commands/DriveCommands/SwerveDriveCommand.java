package frc.robot.commands.DriveCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;

import javax.sql.rowset.spi.TransactionalWriter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.OI;

public class SwerveDriveCommand extends CommandBase {
  private Drivetrain drivetrain;
  private final SlewRateLimiter slewX = new SlewRateLimiter(DriveConstants.kTranslationSlew);
  private final SlewRateLimiter slewY = new SlewRateLimiter(DriveConstants.kTranslationSlew);
  private final SlewRateLimiter slewRot = new SlewRateLimiter(DriveConstants.kRotationSlew);

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = Drivetrain.getInstance();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    OI oi = OI.getInstance();

    Translation2d position = new Translation2d(
        slewX.calculate(-oi.inputTransform(oi.getForward())) * DriveConstants.kMaxSpeedMetersPerSecond,
        slewY.calculate(-oi.inputTransform(oi.getStrafe())) * DriveConstants.kMaxSpeedMetersPerSecond);

    double rotation = slewRot.calculate(-oi.inputTransform(oi.getRotation())) * DriveConstants.kMaxAngularSpeed;
    // double rotation = -oi.inputTransform(oi.getRotation()) *
    // DriveConstants.kMaxAngularSpeed;

    Translation2d centerOfRotation = oi.getCenterOfRotation();

    // double forward = -OI.getInstance().getForward();
    // //not using a deadband because it would not allow continuous motion in
    // certain situations
    // forward = Math.copySign(Math.pow(forward, 2.0), forward); // square joystick
    // input while keeping its sign

    // double strafe = -OI.getInstance().getStrafe();
    // //not using a deadband because it would not allow continuous motion in
    // certain situations
    // strafe = Math.copySign(Math.pow(strafe, 2.0), strafe); // square joystick
    // input while keeping its sign

    // double rotation = -OI.getInstance().getRotation();
    // //not using a deadband because it would not allow continuous motion in
    // certain cituations
    // rotation = Math.copySign(Math.pow(rotation, 2.0), rotation); // square
    // joystick input while keeping its sign

    SmartDashboard.putNumber("position x", position.getX());
    SmartDashboard.putNumber("position y", position.getY());
    SmartDashboard.putNumber("rotation", rotation);

    drivetrain.drive(position, rotation, true, centerOfRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}