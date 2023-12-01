package frc.robot.Shuffleboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.DriveConstants;

public class FieldView {
    private Field2d field;

    private Drivetrain drivetrain = Drivetrain.getInstance();

    private Pose2d[] modulePoses;
    private Pose2d robotPose;

    public FieldView() {
        field = new Field2d();
        modulePoses = new Pose2d[4];
        robotPose = new Pose2d();
    }

    private void updateSwervePoses() {
        if(drivetrain.getPose() != null){
            robotPose = drivetrain.getPose();
        }
        else{
            robotPose = new Pose2d();
        }
        for (int i = 0; i < modulePoses.length; i++) {
            Translation2d updatedPosition = DriveConstants.swerveModuleLocations[i]
                    .rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());

            Rotation2d updatedRotation = new Rotation2d();
            try{
                updatedRotation = drivetrain.getSwerveModuleStates()[i].angle.plus(robotPose.getRotation());
                if(drivetrain.getSwerveModuleStates()[i].speedMetersPerSecond < 0.0) {
                    updatedRotation = updatedRotation.plus(Rotation2d.fromDegrees(180));;
                }
            } catch(NullPointerException e){
            }
            
            modulePoses[i] = new Pose2d(updatedPosition, updatedRotation);
        }
    }

    public void update() {
        updateSwervePoses();

        field.setRobotPose(robotPose);
        field.getObject("Swerve Modules").setPoses(modulePoses);
    }

    public Field2d getField(){
        return field;
    }
}