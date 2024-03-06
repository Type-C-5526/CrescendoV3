// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class GoHome extends Command {

  private ElevatorSubsystem m_Elevator;
  private ShooterPivotSubsystem m_Pivot;
  private ShooterSubsystem m_Shooter;
  private TurretSubsystem m_Turret;
  private Timer m_Timer;

  /** Creates a new GoHome. */
  public GoHome() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = ElevatorSubsystem.getInstance();
    m_Pivot = ShooterPivotSubsystem.getInstance();
    m_Shooter = ShooterSubsystem.getInstance();
    m_Turret = TurretSubsystem.getInstance();
    m_Timer = new Timer();


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_Shooter.disableMotorPID();
    m_Turret.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Turret.isAtSetpoint()){
      m_Pivot.setSetpointInDegrees(-10);
      m_Elevator.disableMotorPID();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Pivot.disablePID();
    m_Timer.stop();
    Superstructure.setRobotStatus(RobotStatus.HOME);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() > 10 || (m_Pivot.atSetpoint() && m_Turret.isAtSetpoint());
  }
}