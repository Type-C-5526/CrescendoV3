package frc.robot.math;

import edu.wpi.first.math.util.Units;

public class Vector {

    private double m_x;
    private double m_y;
    private double m_angle;

    public Vector(double x, double y){
        m_x = x;
        m_y = y;
        m_angle = Math.cos(m_x);
    }

    /**
     * 
     * @param magnitude Magnitude of the Vector
     * @param angle Angle must be in degrees
     * @param a_ Does nothing
     */

    public Vector(double magnitude, double angle, boolean a_){


        m_x = magnitude * Math.cos(Units.degreesToRadians(angle));
        m_y = magnitude * Math.sin(Units.degreesToRadians(angle));
        m_angle = angle;
    }

    public double getX(){
        return m_x;
    }
    public double getY(){
        return m_y;
    }

    public static double scalarProduct(Vector vectorA, Vector vectorB){
        return vectorA.getX() * vectorB.getX() + vectorA.getY() * vectorB.getY();
    }

    public double getAngle(){
        return m_angle;
    }

    /**
     * 
     * @param vectorA
     * @param vectorB
     * @return return angle in degrees
     */
    public static double getAngleBetweenVectors(Vector vectorA, Vector vectorB){
        double angleInRadians = Math.acos(Vector.scalarProduct(vectorA, vectorB)/(vectorA.getMagnitude() * vectorB.getMagnitude()));
        return Units.radiansToDegrees(angleInRadians);
    }

    public double getMagnitude(){
        return Math.sqrt(Math.pow(this.m_x, 2) + Math.pow(this.m_y, 2));
    }

}
