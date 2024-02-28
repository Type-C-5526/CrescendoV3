// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.PoseHelper;

public class AutoAim extends Command {

  private Supplier<Pose2d> m_poseSupplier;
  private boolean isBlue;
  private ShooterPivotSubsystem m_Pivot;

  private double m_quadrant;
  private double m_realtiveQuadrant;
  /** Creates a new AutoAim. */
  public AutoAim(Supplier<Pose2d> _poseSupplier) {
    m_poseSupplier = _poseSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(TurretSubsystem.getInstance());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Pivot = ShooterPivotSubsystem.getInstance();
    m_Pivot.setSetpointInDegrees(30);
    m_Pivot.enablePID();

    TurretSubsystem.getInstance().setSetpoint(0);
    TurretSubsystem.getInstance().enableTurretPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    DriverStation.getAlliance().ifPresent((allianceColor) -> {
        if (allianceColor == Alliance.Blue) {
          isBlue = true;
        }else if (allianceColor == Alliance.Red) {
          isBlue = false;
        }
    });

    Pose2d pose = m_poseSupplier.get();
    
    PoseHelper helper = new PoseHelper( 
      (isBlue) ? Field.BLUE_SPEAKER : Field.RED_SPEAKER, pose.getTranslation());
    double distance = helper.DistanceBetweenPoses();
    double angle = helper.AngleBetweenPoses();
    double heading = pose.getRotation().getDegrees();

    if(heading < 0){
      heading += 360;
    }

    if(heading >= 0 && heading < 90){
      m_quadrant = 0;
      m_realtiveQuadrant = 180;
    }
    else if(heading >= 90 && heading < 180 ){
      m_quadrant = 90;
      m_realtiveQuadrant = 90;
    }
    else if(heading >= 180 && heading < 270 ){
      m_quadrant = 180;
      m_realtiveQuadrant = 0;
    }
    else if(heading >= 270 && heading < 360 ){
      m_quadrant = 270;
      m_realtiveQuadrant = 270;
    }

    double turretSetpoint = (angle + m_realtiveQuadrant - (heading - m_quadrant)) - 90;

    if(turretSetpoint > 90 || turretSetpoint < -90){
      turretSetpoint = 0;
    }


    SmartDashboard.putNumber("Turret Setpoint: ", turretSetpoint);
    SmartDashboard.putBoolean("Is Blue:", isBlue);

    if(m_Pivot.getMeasurment() < 0){
      turretSetpoint = 0;
    }

    TurretSubsystem.getInstance().setSetpoint(TurretSubsystem.getAngleToTicks(turretSetpoint));
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
