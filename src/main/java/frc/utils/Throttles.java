package frc.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.revrobotics.CANSparkMax;

public class Throttles {
    private double limit;
    private double gradient;

    public Throttles(double limit, double gradient){
        this.limit = limit;
        this.gradient = gradient;
    }

    //public void setLimit(SparkMax motor){
    //    motor.getPIDController().setOutputRange(-limit, limit);
    //}
    public void setLimit(WPI_TalonSRX motor){
        //motor.configmax();
    }

    public double applyGradient(double input){
        return input * gradient;
    }
}
