// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveWithLockedHeading extends Command {

  private PIDController m_PID;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private Supplier<Pose2d> m_poseSupplier;
  private DoubleSupplier m_translationX;
  private DoubleSupplier m_translationY;
  private SwerveRequest.FieldCentric m_drive;
  private double m_maxSpeed;
  private double m_maxAngleRate;
  private boolean isBlue = false;

  /** Creates a new DriveWithLockedHeading. */
  public DriveWithLockedHeading(Supplier<Pose2d> _poseSupplier, DoubleSupplier _TranslationX, DoubleSupplier _TranslationY, SwerveRequest.FieldCentric _drive, double _maxAngleRate, double _maxSpeed){
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain);
    
    m_poseSupplier = _poseSupplier;
    m_translationX = _TranslationX;
    m_translationY = _TranslationY;
    m_drive = _drive;
    m_maxSpeed = _maxSpeed;
    m_maxAngleRate = _maxAngleRate;

    m_PID = new PIDController(0.01, 0, 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.getAlliance().ifPresent((allianceColor) -> {
        if (allianceColor == Alliance.Blue) {
          isBlue = true;
        }else if (allianceColor == Alliance.Red) {
          isBlue = true; //TODO CHANGE isBlue to False
        }
    });
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double heading = m_poseSupplier.get().getRotation().getDegrees();

    if(heading < 0){
      heading += 360;
    }

    if(isBlue){
    m_PID.setSetpoint(30);
    }
    else {
      m_PID.setSetpoint(120);
    }
    double output = m_PID.calculate(heading);

    drivetrain.applyRequest(() -> m_drive.withVelocityX(-m_translationX.getAsDouble() * m_maxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_translationY.getAsDouble() * m_maxSpeed) // Drive left with negative X (left)
            .withRotationalRate(output) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true);
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
