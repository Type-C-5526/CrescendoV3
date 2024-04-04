// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class FeedNotes extends Command {
  /** Creates a new FeedNotes. */
  private ConveyorBelt m_conveyor;
  private ShooterSubsystem m_shooter;
  private PivotSubsystem m_pivot;

  public FeedNotes() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_conveyor = ConveyorBelt.getInstance();
    m_shooter = ShooterSubsystem.getInstance();
    m_pivot = PivotSubsystem.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivot.setSetpointInDegrees(20);
    m_pivot.enablePID();
    m_shooter.setSetpoint(-80);
    m_shooter.enableMotorPID();

    m_conveyor.setMotorVelocity(-1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_pivot.atSetpoint() && m_shooter.atSetpoint()){
    Superstructure.setRobotStatus(RobotStatus.AIMED);

    }
    else{
      Superstructure.setRobotStatus(RobotStatus.AIMING);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new GoHome().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
