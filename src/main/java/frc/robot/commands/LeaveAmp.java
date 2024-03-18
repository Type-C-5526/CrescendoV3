// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class LeaveAmp extends Command {
  private PivotSubsystem m_PivotSubsystem = PivotSubsystem.getInstance();
  private ElevatorSubsystem m_Elevator = ElevatorSubsystem.getInstance();
  private TurretSubsystem m_turret = TurretSubsystem.getInstance();
  private BooleanSupplier m_readyToRelease;
  private ConveyorBelt m_conveyor = ConveyorBelt.getInstance();
  private ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();

  private Supplier<Pose2d> m_poseSupplier;
  /** Creates a new HalfExtensionElevator. */
  public LeaveAmp(Supplier<Pose2d> _poseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ElevatorSubsystem.getInstance());
    m_poseSupplier = _poseSupplier;



  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PivotSubsystem.setSetpointInDegrees(-15);
    m_PivotSubsystem.enablePID();

    m_shooter.setSetpoint(-40);
    m_shooter.enableMotorPID();

    Superstructure.setRobotStatus(RobotStatus.LEAVING_IN_AMP);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = m_poseSupplier.get();
    double heading = pose.getRotation().getDegrees();

    if(heading < 0){
      heading += 360;
    }


    if(m_PivotSubsystem.atSetpoint()){ 
      m_Elevator.setSetpointAsPercent(90);
      m_Elevator.enableMotorPID();

    if(m_Elevator.atSetpoint()){
      if(heading > 90 && heading < 270){
        m_turret.setSetpoint(TurretSubsystem.getAngleToTicks(90));
      }
      else{
        m_turret.setSetpoint(TurretSubsystem.getAngleToTicks(-90));
      }
      m_turret.enableTurretPID();
    }

  
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new GoHomeFromAmp().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
