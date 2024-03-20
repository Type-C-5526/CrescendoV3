// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class DeployIntake extends Command {
  /** Creates a new DeployIntake. */
  private Timer m_timer;
  private IntakeSubsystem m_intake;
  private PivotSubsystem m_pivot;
  private ShooterSubsystem m_shooter;
  private ConveyorBelt m_conveyor;


  public DeployIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = IntakeSubsystem.getInstance();
    m_pivot = PivotSubsystem.getInstance();
    m_shooter = ShooterSubsystem.getInstance();
    m_conveyor = ConveyorBelt.getInstance();
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivot.setSetpointInDegrees(-15);
    m_pivot.enablePID();

    if (DriverStation.isAutonomous()) {
      m_intake.setSetpointAsPercent(87);
      m_intake.enableMotorPID();
    }else{
      m_intake.setSetpointAsPercent(90);
      m_intake.enableMotorPID();
    }

    

    m_conveyor.setMotorVelocity(-0.1);

    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_timer.get() > 0.5){
      m_intake.setSpeed(1);
      m_pivot.setSetpointInDegrees(-30);
      m_shooter.setSetpoint(80);
      m_shooter.enableMotorPID();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (!DriverStation.isAutonomous()) {
      new RetractIntake().schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

   if(m_conveyor.hasGamePiece()){
      Superstructure.setRobotStatus(RobotStatus.HAS_GAME_PIECE);
      return true;
    }

    if (DriverStation.isAutonomous() && m_timer.get() > 3) {
      return true;
    }

    return false;
  }
}
