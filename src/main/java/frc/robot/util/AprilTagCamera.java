package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


/** Create a camera */
public class AprilTagCamera {

  private final PhotonCamera m_camera;
  private final Transform3d m_robotToCam;
  private final PhotonPoseEstimator m_estimatedRobotPose;

  private final AprilTagFieldLayout m_fieldLayout;

  public enum Resolution {
    RES_320_240(320, 240),
    RES_640_480(640, 480),
    RES_1280_720(1280, 720),
    RES_1280_800(1280, 800),
    RES_1920_1080(1920, 1080);

    public final int width;
    public final int height;

    private Resolution(int width, int height) {
      this.width = width;
      this.height = height;
    }
  }


  /**
   * Create VisionCamera
   * @param name Name of device
   * @param transform Location on robot in meters
   * @param resolution Resolution used by camera
   * @param fovDiag Diagonal FOV of camera
   */
  public AprilTagCamera(String name, Transform3d transform, Resolution resolution, Rotation2d fovDiag) {
    this.m_camera = new PhotonCamera(name);
    this.m_robotToCam = transform;

    this.m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    m_fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    this.m_estimatedRobotPose = new PhotonPoseEstimator(
      m_fieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      m_camera, 
      transform);

    //m_estimatedRobotPose.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }


  public Optional<EstimatedRobotPose> getEstimatedRobotPose(){
    return m_estimatedRobotPose.update(getPipelineResult());
  }

  public PhotonPipelineResult getPipelineResult(){
    return m_camera.getLatestResult();
  }

  public boolean checkTagsDistance(double distance, int numberOfTags){
    /* */
    var result = getPipelineResult();

    if (result.hasTargets()) {

      if(result.getTargets().size() == numberOfTags){

        if (result.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d()) < distance) {
          
          return true;

        }

      }
    }
    return false;
  }


  public PhotonCamera getCamera(){
    return m_camera;
  }

  public void blinkLEDS(){
    m_camera.setLED(VisionLEDMode.kBlink);
  }

  public void turnOnLEDS(){
    m_camera.setLED(VisionLEDMode.kOn);
  }

  public void turnOffLEDS(){
    m_camera.setLED(VisionLEDMode.kOff);
  }


  public boolean isCameraConnected(){
    return m_camera.isConnected();
  }

  /**
   * Allows user to select the active pipeline index
   * @param index The active pipeline index
   */
  public void setPipelineIndex(int index) {
    m_camera.setPipelineIndex(index);
  }

  /**
   * Get camera to robot transform
   * @return Camera to robot transform
   */
  public Transform3d getTransform() {
    return m_robotToCam;
  }

}