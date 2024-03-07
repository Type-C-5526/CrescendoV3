// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX m_motor;
  private static IntakeSubsystem m_instance;
  
  public IntakeSubsystem() {
    m_motor = new TalonFX(31);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double _speed){
    m_motor.set(_speed);
  }
  public void turnoff(){
    m_motor.set(0);
  }

  public static IntakeSubsystem getInstance(){
    if(m_instance == null){
      m_instance = new IntakeSubsystem();
    }
    return m_instance;
  }
}
