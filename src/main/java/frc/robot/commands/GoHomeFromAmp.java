// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class GoHomeFromAmp extends Command {
  /** Creates a new GoHomeFromAmp. */
  private ElevatorSubsystem m_elevator;
  private PivotSubsystem m_pivot;
  private Timer m_Timer;

  public GoHomeFromAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = ElevatorSubsystem.getInstance();
    m_pivot = PivotSubsystem.getInstance();
    m_Timer = new Timer();
    m_Timer.reset();
    m_Timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setSetpointAsPercent(10);
    m_elevator.enableMotorPID();
    m_pivot.setSetpointInDegrees(-10);
    m_pivot.enablePID();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Superstructure.setRobotStatus(RobotStatus.HOME);
    m_elevator.disableMotorPID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint() || m_Timer.get() > 5;
  }
}
