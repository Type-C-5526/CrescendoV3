package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AprilTagCamera;

public class VisionSubsystem extends SubsystemBase {

    private AprilTagCamera m_aprilTagCamera1;
    private AprilTagCamera m_aprilTagCamera2;

    private static VisionSubsystem m_instance;


    public VisionSubsystem(){

        this.m_aprilTagCamera1 = new AprilTagCamera(
            Vision.CAMERA_A_NAME, 
            Vision.CAMERA_A_LOCATION, 
            null, 
            null);

        m_aprilTagCamera2 = new AprilTagCamera(
            Vision.CAMERA_B_NAME, 
            Vision.CAMERA_B_LOCATION, 
            null, 
            null);

    }




    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        /*SmartDashboard.putBoolean("Is Camera Connected: ", m_aprilTagCamera.isCameraConnected());


        SmartDashboard.putBoolean("Has Targets", m_aprilTagCamera.getPipelineResult().hasTargets());

        if(!m_aprilTagCamera.getPipelineResult().hasTargets()) return;


        SmartDashboard.putBoolean("Is Estimated Present", m_aprilTagCamera.getEstimatedRobotPose().isPresent());*/


        if(Vision.USING_VISION){
            m_aprilTagCamera1.getEstimatedRobotPose().ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose.toPose2d();

                SmartDashboard.putNumber("Estimated Pose X 1", estimatedPose.getX());
                SmartDashboard.putNumber("Estimated Pose y 1", estimatedPose.getY());
                SmartDashboard.putNumber("Estimated Robot Angle 1", estimatedPose.getRotation().getDegrees());

                TunerConstants.DriveTrain.addVisionMeasurement(estimatedPose, estimatedRobotPose.timestampSeconds);
            });

            m_aprilTagCamera2.getEstimatedRobotPose().ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose.toPose2d();

                SmartDashboard.putNumber("Estimated Pose X 2", estimatedPose.getX());
                SmartDashboard.putNumber("Estimated Pose y 2", estimatedPose.getY());
                SmartDashboard.putNumber("Estimated Robot Angle 2", estimatedPose.getRotation().getDegrees());

                TunerConstants.DriveTrain.addVisionMeasurement(estimatedPose, estimatedRobotPose.timestampSeconds);
            });
        }

        
        
    }

    public static VisionSubsystem getInstance(){
        if(m_instance == null){
            m_instance = new VisionSubsystem();
        }
        return m_instance;
    }
}
