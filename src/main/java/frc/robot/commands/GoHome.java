// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class GoHome extends Command {

  private ElevatorSubsystem m_Elevator;
  private PivotSubsystem m_Pivot;
  private ShooterSubsystem m_Shooter;
  private Timer m_Timer;

  /** Creates a new GoHome. */
  public GoHome() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = ElevatorSubsystem.getInstance();
    m_Pivot = PivotSubsystem.getInstance();
    m_Shooter = ShooterSubsystem.getInstance();
    m_Timer = new Timer();


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_Shooter.disableMotorPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        m_Pivot.setSetpointInDegrees(-15);
      if(Superstructure.getRobotStatus() == RobotStatus.LEAVING_IN_AMP){
        m_Elevator.setSetpointAsPercent(20);
        m_Elevator.enableMotorPID();
      }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Timer.stop();
    Superstructure.setRobotStatus(RobotStatus.HOME);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() > 10 || (m_Pivot.atSetpoint());
  }
}
