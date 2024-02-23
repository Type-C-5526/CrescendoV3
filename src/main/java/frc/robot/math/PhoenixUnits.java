package frc.robot.math;

public class PhoenixUnits {
    
    /**
     * Convert Rotations to Degrees
     * 
     * @param _rotations
     * @return Degrees
     */
    public static double getRotationsToDegrees(double _rotations){
        return _rotations * 360;
    }

    /**
     * Convert Rotations To Radians
     * 
     * @param _rotations
     * @return Radians
     */
    public static double getRotationsToRadians(double _rotations){
        return _rotations * 2 * Math.PI;
    }

    /**
     * Convert Degrees to Rotations
     * 
     * @param _degrees
     * @return Sensor Units (Rotations)
     */
    public static double getDegreesToRotations(double _degrees){
        return _degrees / 360;
    }

    /**
     * Convert Radians to Rotations
     * 
     * @param _radians Radians To Convert
     * @return Sensor Units (Rotations)
     */
    public static double getRadiansToRotations(double _radians){
        return _radians / (2 * Math.PI);
    }
}
