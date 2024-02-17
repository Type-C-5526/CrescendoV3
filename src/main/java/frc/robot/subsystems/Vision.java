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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AprilTagCamera;

public class Vision extends SubsystemBase implements AutoCloseable{

    private AprilTagCamera[] m_apriltagCameras;

    private AtomicReference<List<EstimatedRobotPose>> m_estimatedRobotPoses;
    private AtomicReference<List<Integer>> m_visibleTagIDs;

    private Notifier m_cameraNotifier;
    private AprilTagFieldLayout m_fieldLayout;
    private Supplier<Pose2d> m_poseSupplier;
    private VisionSystemSim m_sim;

    private static Vision m_instance;

    public Vision(){

        this.m_apriltagCameras = new AprilTagCamera[Constants.Vision.NUMBER_OF_CAMERAS];
        
        this.m_apriltagCameras[0] = new AprilTagCamera(
            Constants.Vision.CAMERA_A_NAME,
            Constants.Vision.CAMERA_A_LOCATION,
            Constants.Vision.CAMERA_A_RESOLUTION,
            Constants.Vision.CAMERA_A_FOV
        );

        this.m_estimatedRobotPoses = new AtomicReference<List<EstimatedRobotPose>>();
        this.m_visibleTagIDs = new AtomicReference<List<Integer>>();

        // Load AprilTag field layout
        m_fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // PV estimates will always be blue
        m_fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

        // Set field layout for sim
        m_sim.addAprilTags(m_fieldLayout);

        // Setup camera pose estimation threads
        this.m_cameraNotifier = (RobotBase.isReal())
        ? new Notifier(() -> {
            for (var camera : m_apriltagCameras) camera.run();
            updateEstimatedGlobalPoses();
        })
        : new Notifier(() -> {
            if (m_poseSupplier != null) m_sim.update(m_poseSupplier.get());
            for (var camera : m_apriltagCameras) camera.run();
            updateEstimatedGlobalPoses();
        });

        // Set all cameras to primary pipeline
        for (var camera : m_apriltagCameras) camera.setPipelineIndex(0);

        // Add AprilTag cameras to sim
        for (var camera : m_apriltagCameras) m_sim.addCamera(camera.getCameraSim(), camera.getTransform());

        // Start camera thread
        m_cameraNotifier.setName(getName());
        m_cameraNotifier.startPeriodic(Constants.Vision.ROBOT_LOOP_PERIOD);

    }

    /**
     * Update currently estimated robot pose from each camera
     */
    private void updateEstimatedGlobalPoses() {
        List<EstimatedRobotPose> estimatedPoses = new ArrayList<EstimatedRobotPose>();

        List<Integer> visibleTagIDs = new ArrayList<Integer>();
        HashSet<Pose3d> visibleTags = new HashSet<Pose3d>();
        List<Pose2d> loggedPoses = new ArrayList<Pose2d>();
        for (var camera : m_apriltagCameras) {
            var result = camera.getLatestEstimatedPose();
            if (result == null) continue;
            result.targetsUsed.forEach((photonTrackedTarget) -> {
                if (photonTrackedTarget.getFiducialId() == -1) return;
                visibleTagIDs.add(photonTrackedTarget.getFiducialId());
                visibleTags.add(m_fieldLayout.getTagPose(photonTrackedTarget.getFiducialId()).get());
            });
            estimatedPoses.add(result);
            loggedPoses.add(result.estimatedPose.toPose2d());
        }

        /* 
        // Log visible tags and estimated poses
        Logger.recordOutput(getName() + VISIBLE_TAGS_LOG_ENTRY, visibleTags.toArray(new Pose3d[0]));
        Logger.recordOutput(getName() + ESTIMATED_POSES_LOG_ENTRY, loggedPoses.toArray(new Pose2d[0]));
        */
        m_visibleTagIDs.set(visibleTagIDs);
        m_estimatedRobotPoses.set(estimatedPoses);
    }

    /**
     * Set pose supplier for simulation
     * @param poseSupplier Pose supplier from drive subsystem
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        m_poseSupplier = poseSupplier;
    }

    /**
     * Get IDs of currently visible tags
     * @return List of IDs of currently visible tags
     */
    public List<Integer> getVisibleTagIDs() {
        return m_visibleTagIDs.get();
    }

    /**
     * Get currently estimated robot poses from each camera
     * @return List of estimated poses, the timestamp, and targets used to create the estimate
     */
    public List<EstimatedRobotPose> getEstimatedGlobalPoses() {
        return m_estimatedRobotPoses.getAndSet(Collections.emptyList());
    }

    @Override
    public void close() {
        for (var camera : m_apriltagCameras) camera.close();
        m_cameraNotifier.close();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public static Vision getInstance(){
        if(m_instance == null){
            m_instance = new Vision();
        }
        return m_instance;
    }
}
