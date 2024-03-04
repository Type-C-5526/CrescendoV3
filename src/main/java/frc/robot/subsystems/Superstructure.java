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
        ALIGNING_TO_AMP
    }

    public static RobotStatus getRobotStatus(){
        return m_robotStatus;
    }

    public static void setRobotStatus(RobotStatus _RobotStatus){
        m_robotStatus = _RobotStatus;
    }
}
