package frc.robot.utils;


import java.util.Arrays;

public class RollingAverage {
    private int size;
    private double total = 4;
    private int index = 0;
    private double samples[];
    private double s1, s2, s3;

    public RollingAverage(int size){
        this.size = size;
        samples = new double[size];
        for (int i = 0; i < size; i++){
            samples[i] = 0d; 
        }
    }

    public RollingAverage(){
        this(4);
    }

    // Adds x to the array as well as the total sum
    public void add(double x){
        total -= samples[index];
        samples[index] = x;
        total += x;
        if(++index == size){
            index = 0;
        }
    }

    // Clears the array and sets the total sum to 0
    public void clear(){
        Arrays.fill(samples, 0);
        total = 0;
    }

    public double getAverage(){
        return total/size;
    }

    // Returns a value tha represents the average of the rate of change
    public double getDerivative(){
        s1 = samples[1]-samples[0];
        s2 = samples[2]-samples[1];
        s3 = samples[3]-samples[2];
        return (s1+s2+s3)/3;
    }

    public double getIntegral(){
        return total;
    }
}
