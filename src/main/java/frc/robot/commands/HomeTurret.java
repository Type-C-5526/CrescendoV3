// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class HomeTurret extends Command {
  private Timer m_Timer;
  private TurretSubsystem m_Turret;

  /** Creates a new HomeTurret. */
  public HomeTurret() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(TurretSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Turret = TurretSubsystem.getInstance();

    m_Turret.disableTurretPID();
    m_Timer = new Timer();
    m_Timer.reset();
    m_Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   /*  if(!m_Turret.isHome()){
      if(m_Timer.get() < 1.5){
        m_Turret.setMotorVelocity(0.1);
      }else if(m_Timer.get() >= 1.5 && m_Timer.get() < 3)
       m_Turret.setMotorVelocity(-0.1);
    }else{
      
    }
*/
    if(!m_Turret.isHome()){
      if(m_Timer.get() < 5){
        m_Turret.setMotorVelocity(0.05);
      }else if (m_Timer.get() >= 5 && m_Timer.get() < 20) {
        m_Turret.setMotorVelocity(-0.05);
      }
    }else{
      m_Turret.stopTurret();
      m_Turret.resetEncoder();
    }


    /*if(m_Timer.get() < 3){
      
      if(m_Turret.isHome()){
        m_Turret.stopTurret();
      }
      if(!m_Turret.isHome()){
        m_Turret.setMotorVelocity(-0.05);
      }
      else  if(m_Turret.isHome()){
        m_Turret.stopTurret();
        m_Turret.resetEncoder();
      }
 
      if(m_Timer.get()>3 && m_Timer.get() < 6){
        m_Turret.setMotorVelocity(-0.5);
      }
       if(m_Turret.isHome()){
        m_Turret.stopTurret();
      }
      if(!m_Turret.isHome()){
        m_Turret.setMotorVelocity(0.05);
      }
      else {
        m_Turret.stopTurret();
        m_Turret.resetEncoder();

      }
    }*/




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TurretSubsystem.getInstance().stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Turret.isHome();
  }
}
