package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Limelight extends SubsystemBase {

    public abstract boolean hasTarget();

    public abstract Pose2d getBotpose();

    public abstract String getJSONDump();

}
