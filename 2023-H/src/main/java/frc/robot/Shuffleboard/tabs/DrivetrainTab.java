package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainTab extends ShuffleboardTabBase {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private GenericEntry mOdometryX;
    private GenericEntry mOdometryY;
    private GenericEntry mOdometryTheta;
    private GenericEntry mHeading;
    private GenericEntry mSnapToAngleHeading;
    private GenericEntry mUseHeadingCorrection;

    public void createEntries() {
        tab = Shuffleboard.getTab("Drivetrain");

        mOdometryX = tab
                .add("Odometry X", 0.0)
                .getEntry();
        mOdometryY = tab
                .add("Odometry Y", 0.0)
                .getEntry();
        mOdometryTheta = tab
                .add("Odometry Theta", 0.0)
                .getEntry();
        mHeading = tab
                .add("Heading", 0.0)
                .getEntry();
        mSnapToAngleHeading = tab
                .add("SnapToAngleHeading", 0.0)
                .getEntry();
        mUseHeadingCorrection = tab
                .add("Use Heading Correction", false)
                .getEntry();
    }

    @Override
    public void update() {
        mOdometryX.setDouble(drivetrain.getPose().getX());
        mOdometryY.setDouble(drivetrain.getPose().getY());
        mOdometryTheta.setDouble(drivetrain.getPose().getRotation().getDegrees());
        mHeading.setDouble(drivetrain.getHeading());
        // TODO: what exactly is snapToAngleHeading
        mSnapToAngleHeading.setDouble(0.0);
        // TODO: add heading correction in drivetrain
    }
}
