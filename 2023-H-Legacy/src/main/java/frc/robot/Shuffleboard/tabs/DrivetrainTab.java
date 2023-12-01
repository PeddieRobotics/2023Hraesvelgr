package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainTab extends ShuffleboardTabBase {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private GenericEntry mOdometryX, mOdometryY, mOdometryTheta, mHeading, mUseHeadingCorrection,
    mAllowDriving, mModuleRotations1,mModuleRotations2,mModuleRotations3,mModuleRotations4;
    
    public DrivetrainTab(){
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Drivetrain");

        mOdometryX = tab.add("Odometry X", 0.0)
                .getEntry();
        mOdometryY = tab.add("Odometry Y", 0.0)
                .getEntry();
        mOdometryTheta = tab.add("Odometry Theta", 0.0)
                .getEntry();
        mHeading = tab.add("Heading", 0.0)
                .getEntry();
        mModuleRotations1 = tab.add("Module Rotations 1", 0.0)
                .getEntry();
        mModuleRotations2 = tab.add("Module Rotations 2", 0.0)
                .getEntry();
        mModuleRotations3 = tab.add("Module Rotations 3", 0.0)
                .getEntry();
        mModuleRotations4 = tab.add("Module Rotations 4", 0.0)
                .getEntry();
        mUseHeadingCorrection = tab.add("Use Heading Correction", true)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();
        mAllowDriving = tab.add("Allow driving", true)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();

    }

    @Override
    public void update() {
        mOdometryX.setDouble(drivetrain.getPose().getX());
        mOdometryY.setDouble(drivetrain.getPose().getY());
        mOdometryTheta.setDouble(drivetrain.getPose().getRotation().getDegrees());
        mHeading.setDouble(drivetrain.getHeading());
        mModuleRotations1.setDouble(drivetrain.getModuleRotations()[0]);
        mModuleRotations2.setDouble(drivetrain.getModuleRotations()[1]);
        mModuleRotations3.setDouble(drivetrain.getModuleRotations()[2]);
        mModuleRotations4.setDouble(drivetrain.getModuleRotations()[3]);

        drivetrain.setUseHeadingCorrection(mUseHeadingCorrection.getBoolean(true));
        drivetrain.setAllowDriving(mAllowDriving.getBoolean(true));
    }

}
