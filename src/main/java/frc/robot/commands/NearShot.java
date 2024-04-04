// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class NearShot extends Command {

  private PivotSubsystem m_pivot;
  private ShooterSubsystem m_shooter;
  private Timer m_timer;

  /** Creates a new SafeZoneShot. */
  public NearShot() {

    m_pivot = PivotSubsystem.getInstance();
    m_shooter = ShooterSubsystem.getInstance();
    m_timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_pivot.setSetpointInDegrees(52);
    m_pivot.enablePID();


    m_shooter.setSetpoint(-90);
    m_shooter.enableMotorPID();

    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_pivot.atSetpoint() && m_shooter.atSetpoint()) {
      Superstructure.setRobotStatus(RobotStatus.AIMED);

      if (DriverStation.isAutonomous()) {
        m_timer.start();
      ConveyorBelt.getInstance().setMotorVelocity(1);
      }
      
    }else{
      Superstructure.setRobotStatus(RobotStatus.AIMING);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(DriverStation.isAutonomous()){
      m_shooter.disableMotorPID();
    }
    new GoHome().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(DriverStation.isAutonomous() && m_timer.get() > 0.5){
      return true;
    }

    return false;
  }
}
