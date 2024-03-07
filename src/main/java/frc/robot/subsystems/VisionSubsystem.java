package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.util.AprilTagCamera;

public class VisionSubsystem extends SubsystemBase {

    private AprilTagCamera m_aprilTagCamera;

    private static VisionSubsystem m_instance;


    public VisionSubsystem(){

        this.m_aprilTagCamera = new AprilTagCamera(
            Vision.CAMERA_A_NAME, 
            Vision.CAMERA_A_LOCATION, 
            null, 
            null);

    }




    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putBoolean("Is Camera Connected: ", m_aprilTagCamera.isCameraConnected());

        if(m_aprilTagCamera.isCameraConnected()){
            double x = m_aprilTagCamera.getPipelineResult().getBestTarget().getBestCameraToTarget().getX();
            SmartDashboard.putNumber("x april tag: ", x);
        }

        if(!m_aprilTagCamera.getPipelineResult().hasTargets()) return;
        
        if(!m_aprilTagCamera.getEstimatedRobotPose().isEmpty()){
            SmartDashboard.putNumber("Vision Pose X", m_aprilTagCamera.getEstimatedRobotPose().get().estimatedPose.getX());
        }else{
            SmartDashboard.putNumber("Vision Pose X", 0);
        }
        
        
    }

    public static VisionSubsystem getInstance(){
        if(m_instance == null){
            m_instance = new VisionSubsystem();
        }
        return m_instance;
    }
}
