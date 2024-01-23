// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class PIDUtil {
    private final double kP;
    private final double kI;
    private final double kD;
    private double kTolerance;
    private final double kV;
    
    /**
     * 
     * @param _P
     * @param _I
     * @param _D
     * @param _Tolerance
     */
    public PIDUtil(double _P, double _I, double _D, double _Tolerance, double _V) {
        kP = _P;
        kI = _I;
        kD = _D;
        kTolerance = _Tolerance;
        kV = _V;
    }
    
     public PIDUtil(double _P, double _I, double _D, double _Tolerance) {
        kP = _P;
        kI = _I;
        kD = _D;
        kTolerance = _Tolerance;
        kV = 0;

    }

    /**
     * 
     * @param _P
     * @param _I
     * @param _D
     */
    public PIDUtil(double _P, double _I, double _D) {
        kP = _P;
        kI = _I;
        kD = _D;
        kTolerance = 0;
        kV = 0;
    }

    /**
     * 
     * @return
     */
    public double getP(){
        return kP;
    }

    /**
     * 
     * @return
     */
    public double getI(){
        return kI;
    }

    /**
     * 
     * @return
     */
    public double getD(){
        return kD;
    }

    /**
     * 
     * @return
     */
    public double getTolerance(){
        return kTolerance;
    }
    
    public double getV(){
        return kV;
    }


    public void setTolerance(double _Tolerance){
        kTolerance = _Tolerance;

    }

    /**
     * 
     * @param _measurment
     * @param _setpoint
     * @return
     */
    public boolean atSetpoint(double _measurment, double _setpoint){

        double error = _setpoint -_measurment;

        if(kTolerance == 0){
            return (_measurment ==_setpoint) ? true : false;
        }
        else {
            if(error > -kTolerance && error < kTolerance){
                return true;
            }
            else{
                return false;
            } 

        }
        
    }

}
