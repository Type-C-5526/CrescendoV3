// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.util.PIDUtil;

public class Turn90Left extends Command {
  /** Creates a new Turn90Left. */
  public Turn90Left() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Turret.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Turret.getInstance().setSetpoint(Turret.getAngleToTicks(90));
    Turret.getInstance().enableTurretPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Turret.getInstance().disableTurretPID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}