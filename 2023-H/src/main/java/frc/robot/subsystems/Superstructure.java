package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants;
import frc.robot.utils.Logger;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    private Claw claw;
    private Arm arm;
    private double stateDuration;

    public enum SuperstructureState{
        STOWED,
        CUBE_INTAKE_GROUND,
        CONE_INTAKE_GROUND,
        HP_STATION_INTAKE,
        EJECT_L1,
        CUBE_L2,
        CONE_L2,
        CUBE_L3,
        PRE_CONE_INVERTED_L3,
        CONE_INVERTED_L3,
        POST_CONE_INVERTED_L3,
        EJECTING_GAMEPIECE
    }

    SuperstructureState systemState;
    SuperstructureState requestedSystemState;

    public Superstructure(){
        claw = Claw.getInstance();
        arm = Arm.getInstance();

        systemState = SuperstructureState.STOWED;
        requestedSystemState = SuperstructureState.STOWED;

        stateDuration = 0;
    }
    public static Superstructure getInstance() {
        if (superstructure == null) {
            superstructure = new Superstructure();
        }
        return superstructure;
    }

    public void requestState(SuperstructureState request){
        requestedSystemState = request;
    }

    public String getRobotState(){
        return (systemState.toString());
    }

    @Override
    public void periodic(){
        SuperstructureState nextSystemState = systemState;

        switch(systemState){

            //idle state of robot, arm is in stow position
            case STOWED:
                arm.setWristPosition(Constants.WristConstants.kStowedAngle);
                if(arm.getShoulderPosition() > -20){
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kTransitoryAngle, SmartMotionArmSpeed.REGULAR);
                } else {
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kStowedAngle, SmartMotionArmSpeed.SLOW);
                }
                claw.setSpeed(-0.05);

                if(requestedSystemState == SuperstructureState.CUBE_INTAKE_GROUND){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CONE_INTAKE_GROUND){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.EJECT_L1){
                    nextSystemState = requestedSystemState;
                } 
                break;

            case CUBE_INTAKE_GROUND:
                arm.setWristPosition(Constants.WristConstants.kExtendedFloorCubeAngle);
                arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kExtendedFloorCubeAngle, SmartMotionArmSpeed.REGULAR);
                claw.setSpeed(-1);

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.EJECT_L1){
                    nextSystemState = requestedSystemState;
                } else if(claw.isEitherSensor()){
                    nextSystemState = SuperstructureState.STOWED;
                }
                break;

            case CONE_INTAKE_GROUND:
                arm.setWristPosition(Constants.WristConstants.kExtendedFloorConeAngle);
                arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kExtendedFloorConeAngle, SmartMotionArmSpeed.REGULAR);
                claw.setSpeed(-1);

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.EJECT_L1){
                    nextSystemState = requestedSystemState;
                } else if(claw.isBothSensors()){
                    nextSystemState = SuperstructureState.STOWED;
                }
                break;

            case EJECT_L1:
                arm.setWristPosition(Constants.WristConstants.kStowedAngle);
                if(arm.getShoulderPosition() > -20){
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kTransitoryAngle, SmartMotionArmSpeed.REGULAR);
                } else {
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kStowedAngle, SmartMotionArmSpeed.SLOW);
                    arm.setWristPosition(Constants.WristConstants.kL1Angle);
                    
                }
                claw.setSpeed(-0.05);

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.EJECTING_GAMEPIECE){
                    nextSystemState = requestedSystemState;
                }
                break;

            case EJECTING_GAMEPIECE:
                claw.setSpeed(1);

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.CUBE_INTAKE_GROUND){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.CONE_INTAKE_GROUND){
                    nextSystemState = requestedSystemState;
                }
                break;

        }
        systemState = nextSystemState;
    }
}
