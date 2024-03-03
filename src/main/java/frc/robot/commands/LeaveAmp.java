// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class LeaveAmp extends Command {
  private ShooterPivotSubsystem m_PivotSubsystem = ShooterPivotSubsystem.getInstance();
  private ElevatorSubsystem m_Elevator = ElevatorSubsystem.getInstance();
  /** Creates a new HalfExtensionElevator. */
  public LeaveAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ElevatorSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PivotSubsystem.setSetpointInDegrees(30);
    m_PivotSubsystem.enablePID();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_PivotSubsystem.atSetpoint()){
    m_Elevator.setSetpointAsPercent(50);
    m_Elevator.enableMotorPID();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.disableMotorPID();
    m_PivotSubsystem.disablePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
