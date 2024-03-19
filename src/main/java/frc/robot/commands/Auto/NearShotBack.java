// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class NearShotBack extends Command {

  private PivotSubsystem m_pivot;
  private ShooterSubsystem m_shooter;
  private ConveyorBelt m_conveyor;
  private ElevatorSubsystem m_elevator;

  private boolean hasShot;
  private boolean hasToEnd;

  private Timer m_timer;

  /** Creates a new NearShotBack. */
  public NearShotBack() {
    // Use addRequirements() here to declare subsystem dependencies.

    m_elevator = ElevatorSubsystem.getInstance();
    m_conveyor = ConveyorBelt.getInstance();
    m_pivot = PivotSubsystem.getInstance();
    m_shooter = ShooterSubsystem.getInstance();

    m_timer = new Timer();

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setSetpointAsPercent(20);
    m_elevator.enableMotorPID();

    m_pivot.setSetpointInDegrees(0);
    m_pivot.enablePID();

    m_shooter.setSetpoint(-80);
    m_shooter.enableMotorPID();

    hasShot = false;
    hasToEnd = false;

    m_timer.reset();
    m_timer.stop();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_elevator.getPositionInPercent() > 15) {
      m_pivot.setSetpointInDegrees(120);
    }

    if (m_elevator.atSetpoint() && m_shooter.atSetpoint() && m_pivot.atSetpoint()) {
      m_conveyor.setMotorVelocity(1);
      hasShot = true;
      m_timer.start();
    }

    if (hasShot && m_timer.get() > 1) {
      m_pivot.setSetpointInDegrees(-15);

      if (m_pivot.atSetpoint()) {
        hasToEnd = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.disableMotorPID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasToEnd;
  }
}
