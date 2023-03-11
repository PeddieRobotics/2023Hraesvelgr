package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class AngleOverridesTab extends ShuffleboardTabBase {
    private Wrist wrist = Wrist.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();

    private GenericEntry mTransitoryShoulder, mTransitoryWrist, mPreScoreShoulder, mPreScoreWrist,
    mL1Shoulder, mL1Wrist, mL2ConeShoulder, mL2ConeWrist, mL2CubeShoulder, mL2CubeWrist,
    mL3ConeForwardShoulder, mL3ConeForwardWrist, mL3ConeInvertedShoulder, mL3ConeInvertedWrist,
    mL3CubeForwardShoulder, mL3CubeForwardWrist, mStowedShoulder, mStowedWrist, mCompactFloorConeShoulder,
    mCompactFloorConeWrist, mCompactFloorCubeShoulder, mCompactFloorCubeWrist, mExtendedFloorConeShoulder,
    mExtendedFloorConeWrist, mExtendedFloorCubeShoulder, mExtendedFloorCubeWrist,
    mDoubleSSConeShoulder, mDoubleSSConeWrist, mSingleSSShoulder, mSingleSSWrist;

    public void createEntries() {
        tab = Shuffleboard.getTab("Angle Overrides");

        try{
                mStowedShoulder = tab.add("Stowed Shoulder", ShoulderConstants.kStowedAngle)
                .withSize(2, 1)
                .withPosition(0, 0)
                .getEntry();
                mStowedWrist = tab.add("Stowed Wrist", WristConstants.kStowedAngle)
                .withSize(2, 1)
                .withPosition(0, 1)
                .getEntry();
                mSingleSSShoulder = tab.add("Single Substation Shoulder", ShoulderConstants.kSingleSSAngle)
                .withSize(2, 1)
                .withPosition(0, 2)
                        .getEntry();
                mSingleSSWrist = tab.add("Single Substation Wrist", WristConstants.kSingleSSAngle)
                .withSize(2, 1)
                .withPosition(0, 3)
                        .getEntry();
                mDoubleSSConeShoulder = tab.add("Double Substation Cone Shoulder", ShoulderConstants.kDoubleSSConeAngle)
                .withSize(2, 1)
                .withPosition(4, 4)
                        .getEntry();
                mDoubleSSConeWrist = tab.add("Double Substation Cone Wrist", WristConstants.kDoubleSSConeAngle)
                .withSize(2, 1)
                .withPosition(4, 5)
                        .getEntry();


            mL1Shoulder = tab.add("L1 Shoulder", ShoulderConstants.kL1Angle)
            .withSize(2, 1)
            .withPosition(2, 0)
                    .getEntry();
            mL1Wrist = tab.add("L1 Wrist", WristConstants.kL1Angle)
            .withSize(2, 1)
            .withPosition(2, 1)
                    .getEntry();
            mL2ConeShoulder = tab.add("L2 Cone Shoulder", ShoulderConstants.kL2ConeAngle)
            .withSize(2, 1)
            .withPosition(2, 2)
                    .getEntry();
            mL2ConeWrist = tab.add("L2 Cone Wrist", WristConstants.kL2ConeAngle)
            .withSize(2, 1)
            .withPosition(2, 3)
                    .getEntry();
            mL2CubeShoulder = tab.add("L2 Cube Shoulder", ShoulderConstants.kL2CubeAngle)
            .withSize(2, 1)
            .withPosition(2, 4)
                    .getEntry();
            mL2CubeWrist = tab.add("L2 Cube Wrist", WristConstants.kL2CubeAngle)
            .withSize(2, 1)
            .withPosition(2, 5)
                    .getEntry();
            mL3CubeForwardShoulder = tab.add("L3 Cube Forward Shoulder", ShoulderConstants.kL3CubeForwardAngle)
            .withSize(2, 1)
            .withPosition(4, 0)
                    .getEntry();
            mL3CubeForwardWrist = tab.add("L3 Cube Forward Wrist", WristConstants.kL3CubeForwardAngle)
            .withSize(2, 1)
            .withPosition(4, 1)
                    .getEntry();
            mL3ConeForwardShoulder = tab.add("L3 Cone Forward Shoulder", ShoulderConstants.kL3ConeForwardAngle)
            .withSize(2, 1)
            .withPosition(4, 2)
                    .getEntry();
            mL3ConeForwardWrist = tab.add("L3 Cone Forward Wrist", WristConstants.kL3ConeForwardAngle)
            .withSize(2, 1)
            .withPosition(4, 3)
                    .getEntry();
                    mL3ConeInvertedShoulder = tab.add("L3 Cone Inverted Shoulder", ShoulderConstants.kL3ConeInvertedAngle)
                    .withSize(2, 1)
                    .withPosition(4, 2)
                            .getEntry();
                    mL3ConeInvertedWrist = tab.add("L3 Cone Inverted Wrist", WristConstants.kL3ConeInvertedAngle)
                    .withSize(2, 1)
                    .withPosition(4, 3)
                            .getEntry();
            mExtendedFloorConeShoulder = tab.add("Extended Floor Cone Shoulder", ShoulderConstants.kExtendedFloorConeAngle)
            .withSize(2, 1)
            .withPosition(6, 0)
                    .getEntry();
            mExtendedFloorConeWrist = tab.add("Extended Floor Cone Wrist", WristConstants.kExtendedFloorConeAngle)
            .withSize(2, 1)
            .withPosition(6, 1)
                    .getEntry();
            mExtendedFloorCubeShoulder = tab.add("Extended Floor Cube Shoulder", ShoulderConstants.kExtendedFloorCubeAngle)
            .withSize(2, 1)
            .withPosition(6, 2)
                    .getEntry();
            mExtendedFloorCubeWrist = tab.add("Extended Floor Cube Wrist", WristConstants.kExtendedFloorCubeAngle)
            .withSize(2, 1)
            .withPosition(6, 3)
                    .getEntry();
                mPreScoreShoulder = tab.add("Pre-score Shoulder", ShoulderConstants.kPreScoreAngle)
                .withSize(2, 1)
                .withPosition(6, 4)
                .getEntry();
                mPreScoreWrist = tab.add("Pre-score Wrist", WristConstants.kPreScoreAngle)
                .withSize(2, 1)
                .withPosition(6, 5)
                .getEntry();

        } catch(IllegalArgumentException e){
        }
    }

    @Override
    public void update() {
        shoulder.setkStowedAngle(mStowedShoulder.getDouble(ShoulderConstants.kStowedAngle));
        wrist.setkStowedAngle(mStowedWrist.getDouble(WristConstants.kStowedAngle));

        shoulder.setkPreScoreAngle(mPreScoreShoulder.getDouble(ShoulderConstants.kPreScoreAngle));
        wrist.setkPreScoreAngle(mPreScoreWrist.getDouble(WristConstants.kPreScoreAngle));

        shoulder.setkL1Angle(mL1Shoulder.getDouble(ShoulderConstants.kL1Angle));
        wrist.setkL1Angle(mL1Wrist.getDouble(WristConstants.kL1Angle));

        shoulder.setkL2ConeAngle(mL2ConeShoulder.getDouble(ShoulderConstants.kL2ConeAngle));
        wrist.setkL2ConeAngle(mL2ConeWrist.getDouble(WristConstants.kL2ConeAngle));

        shoulder.setkL2CubeAngle(mL2CubeShoulder.getDouble(ShoulderConstants.kL2CubeAngle));
        wrist.setkL2CubeAngle(mL2CubeWrist.getDouble(WristConstants.kL2CubeAngle));

        shoulder.setkL3ConeForwardAngle(mL3ConeForwardShoulder.getDouble(ShoulderConstants.kL3ConeForwardAngle));
        wrist.setkL3ConeForwardAngle(mL3ConeForwardWrist.getDouble(WristConstants.kL3ConeForwardAngle));

        shoulder.setkL3ConeInvertedAngle(mL3ConeInvertedShoulder.getDouble(ShoulderConstants.kL3ConeInvertedAngle));
        wrist.setkL3ConeInvertedAngle(mL3ConeInvertedWrist.getDouble(WristConstants.kL3ConeInvertedAngle));

        shoulder.setkL3CubeForwardAngle(mL3CubeForwardShoulder.getDouble(ShoulderConstants.kL3CubeForwardAngle));
        wrist.setkL3CubeForwardAngle(mL3CubeForwardWrist.getDouble(WristConstants.kL3CubeForwardAngle));

        shoulder.setkExtendedFloorConeAngle(mExtendedFloorConeShoulder.getDouble(ShoulderConstants.kExtendedFloorConeAngle));
        wrist.setkExtendedFloorConeAngle(mExtendedFloorConeWrist.getDouble(WristConstants.kExtendedFloorConeAngle));

        shoulder.setkExtendedFloorCubeAngle(mExtendedFloorCubeShoulder.getDouble(ShoulderConstants.kExtendedFloorCubeAngle));
        wrist.setkExtendedFloorCubeAngle(mExtendedFloorCubeWrist.getDouble(WristConstants.kExtendedFloorCubeAngle));

        shoulder.setkDoubleSSConeAngle(mDoubleSSConeShoulder.getDouble(ShoulderConstants.kDoubleSSConeAngle));
        wrist.setkDoubleSSConeAngle(mDoubleSSConeWrist.getDouble(WristConstants.kDoubleSSConeAngle));

        shoulder.setkSingleSSAngle(mSingleSSShoulder.getDouble(ShoulderConstants.kSingleSSAngle));
        wrist.setkSingleSSAngle(mSingleSSWrist.getDouble(WristConstants.kSingleSSAngle));
        
    }

}
