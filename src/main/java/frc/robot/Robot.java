// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RetractIntake;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.CameraStart;
import frc.robot.util.Telemetry5526;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Telemetry5526 m_telemetry;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    //m_telemetry = new Telemetry5526();

    //CameraStart.startThread();

  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    SmartDashboard.putBoolean("Bypass Color Sensor", Superstructure.isIgnoringColorSensor());
    SmartDashboard.putBoolean("By Pass Aimed", Superstructure.isIgnoringAimed());

    /* 

    if (UseVision) {    

      var visionEstimatedRobotPoses = VisionSubsystem.getInstance().getEstimatedGlobalPoses();

      if (visionEstimatedRobotPoses != null){
        SmartDashboard.putBoolean("Using Vision", true);
        // Add measurment if results exist
        if (!visionEstimatedRobotPoses.isEmpty()){
          // Add vision measurements to pose estimator
          for (var visionEstimatedRobotPose : visionEstimatedRobotPoses) {
            // if (visionEstimatedRobotPose.estimatedPose.toPose2d().getTranslation().getDistance(m_previousPose.getTranslation()) > 1.0) continue;
            m_robotContainer.drivetrain.addVisionMeasurement(visionEstimatedRobotPose.estimatedPose.toPose2d(), visionEstimatedRobotPose.timestampSeconds);
          }
          
        }
      }else{
        SmartDashboard.putBoolean("Using Vision", false);
      }

      */

      

      /* 
      var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

      Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

      if (lastResult.valid) {
        m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
      }
      */
    //}
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //new RetractIntake().schedule();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
