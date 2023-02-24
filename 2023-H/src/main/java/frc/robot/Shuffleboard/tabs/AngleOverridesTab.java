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
    private Claw claw = Claw.getInstance();
    private Wrist wrist = Wrist.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();

    private GenericEntry mL1Shoulder, mL1Wrist, mL2ConeShoulder, mL2ConeWrist, mL2CubeShoulder, mL2CubeWrist,
    mL3ConeShoulder, mL3ConeWrist, mL3CubeShoulder, mL3CubeWrist,
    mStowedShoulder, mStowedWrist, mCompactFloorConeShoulder, mCompactFloorConeWrist,
    mCompactFloorCubeShoulder, mCompactFloorCubeWrist, mExtendedFloorConeShoulder, mExtendedFloorConeWrist,
    mExtendedFloorCubeShoulder, mExtendedFloorCubeWrist;

    public void createEntries() {
        tab = Shuffleboard.getTab("Angle Overrides");

        try{
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
            mStowedShoulder = tab.add("L3 Stowed Shoulder", ShoulderConstants.kStowedAngle)
                    .getEntry();
            mStowedWrist = tab.add("Stowed Wrist", WristConstants.kStowedAngle)
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
        } catch(IllegalArgumentException e){
        }
    }

    @Override
    public void update() {

    }

}
