// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  
  private static ShooterSubsystem m_instance;
  private TalonFX m_motor1;
  private TalonFXConfiguration m_motor1Config;
  private VelocityVoltage m_VelocityVoltage;
  private double m_setpoint;
  private boolean m_enabled;
  private NeutralOut m_break;
  
  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    m_setpoint = 0;
    m_enabled = false;
    m_break = new NeutralOut();
    m_motor1 = new TalonFX(Constants.Shooter.talonFXShooterID, Constants.Shooter.canbus);
    m_motor1Config = new TalonFXConfiguration();
    m_motor1Config.Slot0.kP = Constants.Shooter.pidConstants.getP();
    m_motor1Config.Slot0.kI = Constants.Shooter.pidConstants.getI();
    m_motor1Config.Slot0.kD = Constants.Shooter.pidConstants.getD();
    m_motor1Config.Slot0.kV = Constants.Shooter.pidConstants.getV();

    m_motor1Config.Voltage.PeakForwardVoltage = 12;
    m_motor1Config.Voltage.PeakReverseVoltage = -12;
    m_motor1.getConfigurator().apply(m_motor1Config);
    
    m_VelocityVoltage = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
    
  }

  public void setSetpoint(double _Setpoint){
    m_setpoint = _Setpoint;
  }

  public void enableMotorPID() {
    m_enabled = true;
  }

  public void disableMotorPID() {
    m_enabled = false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_enabled){
      m_motor1.setControl(m_VelocityVoltage.withVelocity(m_setpoint));
    }
    else {
      m_motor1.setControl(m_break);
    }



    SmartDashboard.putBoolean("At Setpoint", atSetpoint());
    SmartDashboard.putNumber("Motor Velocity", m_motor1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Error", getError());
    SmartDashboard.putNumber("Setpoint", m_setpoint);


  }

  public double getError(){
    return m_setpoint - m_motor1.getVelocity().getValueAsDouble();
  }

  public boolean atSetpoint(){
    return Constants.Shooter.pidConstants.atSetpoint(m_motor1.getVelocity().getValueAsDouble(), m_setpoint);
  }

  public static ShooterSubsystem getInstance(){
    if(m_instance == null){
      m_instance = new ShooterSubsystem();
    }
    return m_instance;
  }

}
