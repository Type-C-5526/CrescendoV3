// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class RetractIntake extends Command {
  /** Creates a new RetractIntake. */
  private IntakeSubsystem m_intake;
  private PivotSubsystem m_pivot;
  private ShooterSubsystem m_shooter;
  private Timer m_timer;
  public RetractIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = IntakeSubsystem.getInstance();
    m_pivot = PivotSubsystem.getInstance();
    m_shooter = ShooterSubsystem.getInstance();
    
    m_timer = new Timer();
    m_timer.reset();
    m_timer.start();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivot.setSetpointInDegrees(-10);
    m_pivot.enablePID();
    m_shooter.disableMotorPID();
    m_intake.setSpeed(0);
    ElevatorSubsystem.getInstance().disableMotorPID();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_pivot.atSetpoint()){
      m_intake.setSetpointAsPercent(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Superstructure.setRobotStatus(RobotStatus.HOME);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.atSetpoint() || m_timer.get() > 2;
  }
}
