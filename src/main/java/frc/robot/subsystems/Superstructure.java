package frc.robot.subsystems;

public class Superstructure {
    private static RobotStatus m_robotStatus = RobotStatus.AIMED;
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
        HAS_GAME_PIECE
    }

    public static RobotStatus getRobotStatus(){
        return m_robotStatus;
    }

    public static void setRobotStatus(RobotStatus _RobotStatus){
        m_robotStatus = _RobotStatus;
    }
}
