package frc.robot.commands.DriveCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;

import javax.sql.rowset.spi.TransactionalWriter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants;
import frc.robot.utils.OI;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.Robot;

public class SwerveDriveCommand extends CommandBase {
  private Drivetrain drivetrain;
  private OI oi = OI.getInstance();
  private final SlewRateLimiter slewX = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
  private final SlewRateLimiter slewY = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
  private final SlewRateLimiter slewRot = new SlewRateLimiter(DriveConstants.kRotationSlewRate);

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

    Translation2d centerOfRotation = oi.getCenterOfRotation();

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