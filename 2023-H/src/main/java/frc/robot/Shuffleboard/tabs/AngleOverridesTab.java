package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class AngleOverridesTab extends ShuffleboardTabBase {
    private Wrist wrist = Wrist.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();

    private GenericEntry mTransitoryShoulder, mTransitoryWrist, mLLSeekShoulder, mLLSeekWrist,
    mL1Shoulder, mL1Wrist, mL2ConeShoulder, mL2ConeWrist, mL2CubeShoulder, mL2CubeWrist,
    mL3ConeShoulder, mL3ConeWrist, mL3CubeShoulder, mL3CubeWrist,
    mStowedShoulder, mStowedWrist, mCompactFloorConeShoulder, mCompactFloorConeWrist,
    mCompactFloorCubeShoulder, mCompactFloorCubeWrist, mExtendedFloorConeShoulder, mExtendedFloorConeWrist,
    mExtendedFloorCubeShoulder, mExtendedFloorCubeWrist, mDoubleSSConeShoulder, mDoubleSSConeWrist,
    mSingleSSShoulder, mSingleSSWrist;

    public void createEntries() {
        tab = Shuffleboard.getTab("Angle Overrides");

        try{
                mStowedShoulder = tab.add("Stowed Shoulder", ShoulderConstants.kStowedAngle)
                .getEntry();
                mStowedWrist = tab.add("Stowed Wrist", WristConstants.kStowedAngle)
                .getEntry();
                mTransitoryShoulder = tab.add("Transitory Shoulder", ShoulderConstants.kTransitoryAngle)
                .getEntry();
                mTransitoryWrist = tab.add("Transitory Wrist", WristConstants.kTransitoryAngle)
                .getEntry();
                mLLSeekShoulder = tab.add("LL Seek Shoulder", ShoulderConstants.kLLSeekAngle)
                .getEntry();
                mLLSeekWrist = tab.add("LL Seek Wrist", WristConstants.kLLSeekAngle)
                .getEntry();
            mL1Shoulder = tab.add("L1 Shoulder", ShoulderConstants.kL1Angle)
                    .getEntry();
            mL1Wrist = tab.add("L1 Wrist", WristConstants.kL1Angle)
                    .getEntry();
            mL2ConeShoulder = tab.add("L2 Cone Shoulder", ShoulderConstants.kL2ConeAngle)
                    .getEntry();
            mL2ConeWrist = tab.add("L2 Cone Wrist", WristConstants.kL2ConeAngle)
                    .getEntry();
            mL2CubeShoulder = tab.add("L2 Cube Shoulder", ShoulderConstants.kL2CubeAngle)
                    .getEntry();
            mL2CubeWrist = tab.add("L2 Cube Wrist", WristConstants.kL2CubeAngle)
                    .getEntry();
            mL3CubeShoulder = tab.add("L3 Cube Shoulder", ShoulderConstants.kL3CubeForwardAngle)
                    .getEntry();
            mL3CubeWrist = tab.add("L3 Cube Wrist", WristConstants.kL3CubeForwardAngle)
                    .getEntry();
            mL3ConeShoulder = tab.add("L3 Cone Shoulder", ShoulderConstants.kL3ConeAngle)
                    .getEntry();
            mL3ConeWrist = tab.add("L3 Cone Wrist", WristConstants.kL3ConeAngle)
                    .getEntry();
            mCompactFloorConeShoulder = tab.add("Compact Floor Cone Shoulder", ShoulderConstants.kCompactFloorConeAngle)
                    .getEntry();
            mCompactFloorConeWrist = tab.add("Compact Floor Cone Wrist", WristConstants.kCompactFloorConeAngle)
                    .getEntry();
            mCompactFloorCubeShoulder = tab.add("Compact Floor Cube Shoulder", ShoulderConstants.kCompactFloorCubeAngle)
                    .getEntry();
            mCompactFloorCubeWrist = tab.add("Compact Floor Cube Wrist", WristConstants.kCompactFloorCubeAngle)
                    .getEntry();
            mExtendedFloorConeShoulder = tab.add("Extended Floor Cone Shoulder", ShoulderConstants.kExtendedFloorConeAngle)
                    .getEntry();
            mExtendedFloorConeWrist = tab.add("Extended Floor Cone Wrist", WristConstants.kExtendedFloorConeAngle)
                    .getEntry();
            mExtendedFloorCubeShoulder = tab.add("Extended Floor Cube Shoulder", ShoulderConstants.kExtendedFloorCubeAngle)
                    .getEntry();
            mExtendedFloorCubeWrist = tab.add("Extended Floor Cube Wrist", WristConstants.kExtendedFloorCubeAngle)
                    .getEntry();
                mDoubleSSConeShoulder = tab.add("Double Substation Cone Shoulder", ShoulderConstants.kDoubleSSConeAngle)
                .getEntry();
                mDoubleSSConeWrist = tab.add("Double Substation Cone Wrist", WristConstants.kDoubleSSConeAngle)
                .getEntry();
                mSingleSSShoulder = tab.add("Single Substation Shoulder", ShoulderConstants.kSingleSSAngle)
                .getEntry();
                mSingleSSWrist = tab.add("Single Substation Wrist", WristConstants.kSingleSSAngle)
                .getEntry();
        } catch(IllegalArgumentException e){
        }
    }

    @Override
    public void update() {
        shoulder.setkStowedAngle(mStowedShoulder.getDouble(ShoulderConstants.kStowedAngle));
        wrist.setkStowedAngle(mStowedWrist.getDouble(WristConstants.kStowedAngle));

        shoulder.setkTransitoryAngle(mTransitoryShoulder.getDouble(ShoulderConstants.kTransitoryAngle));
        wrist.setkTransitoryAngle(mTransitoryWrist.getDouble(WristConstants.kTransitoryAngle));

        shoulder.setkLLSeekAngle(mLLSeekShoulder.getDouble(ShoulderConstants.kLLSeekAngle));
        wrist.setkLLSeekAngle(mLLSeekWrist.getDouble(WristConstants.kLLSeekAngle));

        shoulder.setkL1Angle(mL1Shoulder.getDouble(ShoulderConstants.kL1Angle));
        wrist.setkL1Angle(mL1Wrist.getDouble(WristConstants.kL1Angle));

        shoulder.setkL2ConeAngle(mL2ConeShoulder.getDouble(ShoulderConstants.kL2ConeAngle));
        wrist.setkL2ConeAngle(mL2ConeWrist.getDouble(WristConstants.kL2ConeAngle));

        shoulder.setkL2CubeAngle(mL2CubeShoulder.getDouble(ShoulderConstants.kL2CubeAngle));
        wrist.setkL2CubeAngle(mL2CubeWrist.getDouble(WristConstants.kL2CubeAngle));

        shoulder.setkL3ConeAngle(mL3ConeShoulder.getDouble(ShoulderConstants.kL3ConeAngle));
        wrist.setkL3ConeAngle(mL3ConeWrist.getDouble(WristConstants.kL3ConeAngle));

        shoulder.setkL3CubeForwardAngle(mL3CubeShoulder.getDouble(ShoulderConstants.kL3CubeForwardAngle));
        wrist.setkL3CubeForwardAngle(mL3CubeWrist.getDouble(WristConstants.kL3CubeForwardAngle));

        shoulder.setkCompactFloorConeAngle(mCompactFloorConeShoulder.getDouble(ShoulderConstants.kCompactFloorConeAngle));
        wrist.setkCompactFloorConeAngle(mCompactFloorConeWrist.getDouble(WristConstants.kCompactFloorConeAngle));

        shoulder.setkCompactFloorCubeAngle(mCompactFloorCubeShoulder.getDouble(ShoulderConstants.kCompactFloorCubeAngle));
        wrist.setkCompactFloorCubeAngle(mCompactFloorCubeWrist.getDouble(WristConstants.kCompactFloorCubeAngle));

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
