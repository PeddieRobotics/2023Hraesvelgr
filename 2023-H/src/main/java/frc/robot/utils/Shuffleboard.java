package frc.robot.utils;

public class Shuffleboard {
    public static Shuffleboard shuffleboard;

    public Shuffleboard(){}

    public static Shuffleboard getInstance(){
        if(shuffleboard == null){
            shuffleboard = new Shuffleboard();
        }
        return shuffleboard;
    }

    public void putShuffleboard(){

    }

    public void updateFromShuffleboard(){
        
    }
    
}
