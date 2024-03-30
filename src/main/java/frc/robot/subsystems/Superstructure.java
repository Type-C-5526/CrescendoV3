package frc.robot.subsystems;

public class Superstructure {
    private static RobotStatus m_robotStatus = RobotStatus.HOME;

    private static boolean m_isChasisAimed = false;

    public static void setChasisAimed(boolean aimed){
        m_isChasisAimed = aimed;
    }

    public static boolean isChasisAimed(){
        return m_isChasisAimed;
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
