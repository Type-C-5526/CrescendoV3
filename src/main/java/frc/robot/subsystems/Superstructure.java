package frc.robot.subsystems;

public class Superstructure {
    private static RobotStatus m_robotStatus = RobotStatus.HOME;

    private static boolean m_isChasisAimed = false;
    private static boolean m_ignoreAimed = false;
    private static boolean m_ignoreColorSensor = false;

    public static void setChasisAimed(boolean aimed){
        m_isChasisAimed = aimed;
    }

    public static void setIgnoreAimed(boolean ignore){
       m_ignoreAimed =  ignore;
    }

    public static boolean isIgnoringAimed(){
        return m_ignoreAimed;
    }

    public static boolean isIgnoringColorSensor(){
        return m_ignoreColorSensor;
    }

    public static void setIgnoreColorSensor(boolean ignore){
       m_ignoreColorSensor =  ignore;
    }

    public static boolean isChasisAimed(){
        return m_isChasisAimed;
    }

    public static void switchIgnoreAimed(){
        m_ignoreAimed = !m_ignoreAimed;
    }

    public static void switchIgnoreColorSensor(){
        m_ignoreColorSensor = !m_ignoreColorSensor;
    }

    public static enum RobotStatus{
        AIMING,
        AIMED,
        PICKING_FROM_SOURCE,
        PICKING_FROM_FLOOR,
        SHOOTING,
        HOME,
        ELEVATING,
        SCORING_IN_AMP,
        LEAVING_IN_AMP,
        HAS_GAME_PIECE,
        CANT_AIM
    }

    public static RobotStatus getRobotStatus(){
        return m_robotStatus;
    }

    public static void setRobotStatus(RobotStatus _RobotStatus){
        m_robotStatus = _RobotStatus;
    }
}
