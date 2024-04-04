package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AprilTagCamera;

public class VisionSubsystem extends SubsystemBase {

    private AprilTagCamera m_aprilTagCamera1;

    private static VisionSubsystem m_instance;


    public VisionSubsystem(){

        this.m_aprilTagCamera1 = new AprilTagCamera(
            Vision.CAMERA_A_NAME, 
            Vision.CAMERA_A_LOCATION, 
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
        }

        
        
    }

    public AprilTagCamera getAprilTagCamera(){
      return m_aprilTagCamera1;
    }


    public static VisionSubsystem getInstance(){
        if(m_instance == null){
            m_instance = new VisionSubsystem();
        }
        return m_instance;
    }
}