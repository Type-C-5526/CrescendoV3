// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  private ShooterSubsystem m_shooter;
  private ConveyorBelt m_conveyor;
  private ShooterPivotSubsystem m_pivot;

  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_conveyor = ConveyorBelt.getInstance();
    m_shooter = ShooterSubsystem.getInstance();
    m_pivot = ShooterPivotSubsystem.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setSetpoint(-80);
    m_shooter.enableMotorPID();
    m_pivot.setSetpointInDegrees(30);
    m_pivot.enablePID();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_shooter.atSetpoint() && m_pivot.atSetpoint()){


      m_conveyor.setMotorVelocity(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      //Conveyor.getInstance().stop();
      ShooterSubsystem.getInstance().disableMotorPID();
      m_conveyor.setMotorVelocity(0);
      m_shooter.disableMotorPID();
      m_pivot.disablePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}