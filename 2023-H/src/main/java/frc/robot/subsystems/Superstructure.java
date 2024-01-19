package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Logger;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    private Claw claw;
    private Arm arm;
    private Shoulder shoulder;
    private double stateDuration;
    private double internalStateTimer;

    public enum SuperstructureState{
        STOWED,
        CUBE_INTAKE_GROUND,
        CONE_INTAKE_GROUND,
        HP_STATION_INTAKE,
        SCORE_L1,
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
        shoulder = Shoulder.getInstance();

        systemState = SuperstructureState.STOWED;
        requestedSystemState = SuperstructureState.STOWED;

        stateDuration = 0;
        internalStateTimer = 0;
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
                } else if (requestedSystemState == SuperstructureState.HP_STATION_INTAKE){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SCORE_L1){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUBE_L2){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CONE_L2){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUBE_L3){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PRE_CONE_INVERTED_L3){
                    nextSystemState = requestedSystemState;
                }
                break;

            case CUBE_INTAKE_GROUND:
                arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kExtendedFloorCubeAngle, SmartMotionArmSpeed.REGULAR);
                claw.setSpeed(-1);
                if(arm.isShoulderBelowAngle(Constants.ShoulderConstants.kExtendedFloorConeAngle + 20)){
                    arm.setWristPosition(Constants.WristConstants.kExtendedFloorCubeAngle);
                } else {
                    arm.setWristPosition(Constants.WristConstants.kStowedAngle);
                }

                if(!claw.hasGamepiece()){
                    internalStateTimer=Timer.getFPGATimestamp();
                } 

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.CONE_INTAKE_GROUND){
                    nextSystemState = requestedSystemState;
                } else if(claw.isEitherSensor() && Timer.getFPGATimestamp() - internalStateTimer>.3){
                    nextSystemState = SuperstructureState.STOWED;
                    requestedSystemState = SuperstructureState.STOWED;
                    break;
                }
                break;

            case CONE_INTAKE_GROUND:
                arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kExtendedFloorConeAngle, SmartMotionArmSpeed.REGULAR);
                claw.setSpeed(-1);
                if(arm.isShoulderBelowAngle(Constants.ShoulderConstants.kExtendedFloorConeAngle + 20)){
                    arm.setWristPosition(Constants.WristConstants.kExtendedFloorConeAngle);
                } else {
                    arm.setWristPosition(Constants.WristConstants.kStowedAngle);
                }

                if(!claw.hasGamepiece()){
                    internalStateTimer=Timer.getFPGATimestamp();
                } 

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.CUBE_INTAKE_GROUND){
                    nextSystemState = requestedSystemState;
                } else if(claw.isBothSensors() && Timer.getFPGATimestamp() - internalStateTimer>.3){
                    nextSystemState = SuperstructureState.STOWED;
                    requestedSystemState = SuperstructureState.STOWED;
                    break;
                }
                break;

            case EJECTING_GAMEPIECE:
                claw.setSpeed(1);

                if(requestedSystemState == SuperstructureState.STOWED || !claw.hasGamepiece()){
                    nextSystemState = SuperstructureState.STOWED;
                    requestedSystemState = SuperstructureState.STOWED;
                } else if(requestedSystemState == SuperstructureState.CUBE_INTAKE_GROUND){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.CONE_INTAKE_GROUND){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.HP_STATION_INTAKE){
                    nextSystemState = requestedSystemState;
                }
                break;

            case HP_STATION_INTAKE:
                claw.setSpeed(-.5);  
                if(arm.isShoulderAboveAngle(-20)){
                    arm.setWristPosition(Constants.WristConstants.kStowedAngle);
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kTransitoryAngle, SmartMotionArmSpeed.REGULAR);
                } else {
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kStowedAngle, SmartMotionArmSpeed.SLOW);
                    arm.setWristPosition(Constants.WristConstants.kL1Angle);
                }
                arm.setWristPosition(Constants.WristConstants.kSingleSSConeAngle);

                if(!claw.hasGamepiece()){
                    internalStateTimer=Timer.getFPGATimestamp();
                } 

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                } else if(claw.isEitherSensor() && Timer.getFPGATimestamp()-internalStateTimer>0.3){
                    nextSystemState = SuperstructureState.STOWED;
                    requestedSystemState = SuperstructureState.STOWED;
                }
                break;

            case SCORE_L1:
                if(arm.getShoulderPosition() > -20){
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kTransitoryAngle, SmartMotionArmSpeed.REGULAR);
                    arm.setWristPosition(Constants.WristConstants.kStowedAngle);
                } else {
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kStowedAngle, SmartMotionArmSpeed.SLOW);
                    arm.setWristPosition(Constants.WristConstants.kL1Angle);
                }
                claw.setSpeed(-0.05);

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.EJECTING_GAMEPIECE){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUBE_L2){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CONE_L2){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUBE_L3){
                    nextSystemState = requestedSystemState;
                } 
                break;

            case CUBE_L2: 
                arm.setWristPosition(Constants.WristConstants.kStowedAngle);
                arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kL2CubeAngle, SmartMotionArmSpeed.REGULAR);
                if(arm.isShoulderAboveAngle(-20)){
                    arm.setWristPosition(Constants.WristConstants.kL2CubeAngle);
                }

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.EJECTING_GAMEPIECE){
                    nextSystemState = requestedSystemState;
                } 
                break;

            case CONE_L2:
                arm.setWristPosition(Constants.WristConstants.kStowedAngle);
                arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kL2ConeAngle, SmartMotionArmSpeed.REGULAR);
                if(arm.isShoulderAboveAngle(-30)){
                    arm.setWristPosition(Constants.WristConstants.kL2ConeAngle);
                }

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                }else if (requestedSystemState == SuperstructureState.EJECTING_GAMEPIECE){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SCORE_L1){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CONE_L2){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUBE_L3){
                    nextSystemState = requestedSystemState;
                } 
                break;

            case CUBE_L3:
                arm.setWristPosition(Constants.WristConstants.kStowedAngle);
                arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kL3CubeForwardAngle, SmartMotionArmSpeed.REGULAR);
                if(arm.isShoulderAboveAngle(-20)){
                    arm.setWristPosition(Constants.WristConstants.kL3CubeForwardAngle);
                }

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.EJECTING_GAMEPIECE){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SCORE_L1){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUBE_L2){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CONE_L2){
                    nextSystemState = requestedSystemState;
                } 
                break;

            case PRE_CONE_INVERTED_L3:
                arm.setWristPosition(Constants.WristConstants.kHomeAngle);
                if(arm.isShoulderBelowAngle(65)){
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kL3ConeInvertedAngle, SmartMotionArmSpeed.FAST);
                } else {
                    shoulder.setSlowSmartMotionParameters(Constants.ShoulderConstants.kSmartMotionSlowSetpointTol, 
                                                          Constants.ShoulderConstants.kSmartMotionSlowMinVel, 3000, 2000);
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kL3ConeInvertedAngle, SmartMotionArmSpeed.SLOW);
                }

                if(arm.isShoulderAboveAngle(75.0)){
                    arm.setWristPosition(Constants.WristConstants.kL3ConeInvertedAngle);
                }

                if(requestedSystemState == SuperstructureState.STOWED || requestedSystemState == SuperstructureState.SCORE_L1 || requestedSystemState == SuperstructureState.CUBE_L2 
                    || requestedSystemState == SuperstructureState.CONE_L2 || requestedSystemState == SuperstructureState.CUBE_L3){
                    shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
                                                          ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kSmartMotionSlowMaxVel, ShoulderConstants.kSmartMotionSlowMaxAccel);
                    if(arm.isShoulderAboveAngle(65.0)) {
                        nextSystemState = SuperstructureState.POST_CONE_INVERTED_L3;
                        arm.holdShoulderPosition();
                    } else {
                        nextSystemState = requestedSystemState;
                    }

                }
                
                if (arm.isShoulderAtAngle(Constants.ShoulderConstants.kL3ConeInvertedAngle) && arm.isWristAtAngle(Constants.WristConstants.kL3ConeInvertedAngle)){
                    nextSystemState = requestedSystemState = SuperstructureState.CONE_INVERTED_L3;
                    arm.holdShoulderPosition();
                }
                break;
                
            case CONE_INVERTED_L3:

                if(requestedSystemState == SuperstructureState.EJECTING_GAMEPIECE){
                    claw.setSpeed(1);
                }

                if(claw.hasGamepiece()){
                    internalStateTimer=Timer.getFPGATimestamp();
                } 

                if(requestedSystemState == SuperstructureState.STOWED){
                    nextSystemState = SuperstructureState.POST_CONE_INVERTED_L3;
                } else if (requestedSystemState == SuperstructureState.SCORE_L1){
                    nextSystemState = SuperstructureState.POST_CONE_INVERTED_L3;
                } else if (requestedSystemState == SuperstructureState.CUBE_L2){
                    nextSystemState = SuperstructureState.POST_CONE_INVERTED_L3;
                } else if (requestedSystemState == SuperstructureState.CONE_L2){
                    nextSystemState = SuperstructureState.POST_CONE_INVERTED_L3;
                } else if (requestedSystemState == SuperstructureState.CUBE_L3){
                    nextSystemState = SuperstructureState.POST_CONE_INVERTED_L3;
                } else if (Timer.getFPGATimestamp() - internalStateTimer >.2 && !claw.hasGamepiece()){
                    nextSystemState = SuperstructureState.POST_CONE_INVERTED_L3;
                    requestedSystemState = SuperstructureState.STOWED;
                } 
                break;

            case POST_CONE_INVERTED_L3:
                shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
                                                    ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kSmartMotionSlowMaxVel, ShoulderConstants.kSmartMotionSlowMaxAccel);
                arm.setWristPosition(Constants.WristConstants.kStowedAngle);
                claw.setSpeed(0);
                if(arm.isShoulderAboveAngle(65)){
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kL3CubeForwardAngle, SmartMotionArmSpeed.SLOW);
                } else {
                    arm.setShoulderPositionSmartMotion(Constants.ShoulderConstants.kL3CubeForwardAngle, SmartMotionArmSpeed.SLOW);
                }

                if(arm.isShoulderBelowAngle(20)){
                    if(requestedSystemState == SuperstructureState.STOWED){
                        nextSystemState = SuperstructureState.STOWED;
                    } else if(requestedSystemState == SuperstructureState.CUBE_INTAKE_GROUND){
                        nextSystemState = requestedSystemState;
                    } else if (requestedSystemState == SuperstructureState.CONE_INTAKE_GROUND){
                        nextSystemState = requestedSystemState;
                    } else if (requestedSystemState == SuperstructureState.HP_STATION_INTAKE){
                        nextSystemState = requestedSystemState;
                    }   else if (requestedSystemState == SuperstructureState.SCORE_L1){
                        nextSystemState = requestedSystemState;
                    } else if (requestedSystemState == SuperstructureState.CUBE_L2){
                        nextSystemState = requestedSystemState;
                    } else if (requestedSystemState == SuperstructureState.CONE_L2){
                        nextSystemState = requestedSystemState;
                    } else if (requestedSystemState == SuperstructureState.CUBE_L3){
                        nextSystemState = requestedSystemState;
                    }
                }
                break;

        }

        systemState = nextSystemState;
    }
}
