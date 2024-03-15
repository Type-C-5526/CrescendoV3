// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends Command {
  /** Creates a new DeployIntake. */
  private IntakeSubsystem m_intake;
  public DeployIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = IntakeSubsystem.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setSetpointAsPercent(100);
    m_intake.enableMotorPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_intake.atSetpoint()){
      m_intake.setSpeed(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_intake.setSpeed(0);
      m_intake.setSetpointAsPercent(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
