// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX m_IntakeMotor;
  private CANSparkMax m_rackMotor; 
  private PIDController m_pidController;
  private double m_setpoint;
  private boolean m_enabled;
  private RelativeEncoder m_encoder;

  private double m_maxOutputForward = 0.2;
  private double m_maxOutputReverse = -0.2;

  private DutyCycleOut m_dutycycle = new DutyCycleOut(0);




  private static IntakeSubsystem m_instance;
  
  public IntakeSubsystem() {
    m_IntakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);

    TalonFXConfiguration m_configs = new TalonFXConfiguration();
    m_IntakeMotor.getConfigurator().apply(m_configs);
    
    m_rackMotor = new CANSparkMax(Constants.Intake.RACK_MOTOR_ID, MotorType.kBrushless);

    m_setpoint = 0;
    m_enabled = false;


    m_encoder = m_rackMotor.getEncoder();
    m_encoder.setPosition(0.0);



    m_pidController = new PIDController(Constants.Intake.TurretPIDConstants.getP(), Constants.Intake.TurretPIDConstants.getI(), Constants.Intake.TurretPIDConstants.getD());
    m_pidController.setTolerance(50);

    m_rackMotor.restoreFactoryDefaults();
    m_rackMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("Intake Output:", getMeasurment());

    if(m_enabled){
      double output = m_pidController.calculate(getMeasurment());
      

      if(output > m_maxOutputForward){
        output = m_maxOutputForward;
      }
      else if(output < m_maxOutputReverse){
        output = m_maxOutputReverse;
      }

      m_rackMotor.set(output);
    }
    else {
      m_rackMotor.stopMotor();
    }
  }

  public void setSpeed(double _speed){
    m_IntakeMotor.setControl(m_dutycycle.withOutput(_speed));
  }
  public void turnoff(){
    m_IntakeMotor.set(0);
  }


  public void setSetpointAsPercent(double _Setpoint){
    m_setpoint = _Setpoint * -2.6317;
    m_pidController.setSetpoint(m_setpoint);
  }

  public void enableMotorPID() {
    m_enabled = true;
  }

  public void disableMotorPID() {
    m_enabled = false;
  }
  public void setSetpoint(double _Setpoint){
    m_setpoint = _Setpoint;
    m_pidController.setSetpoint(_Setpoint);
  }

  public boolean atSetpoint(){
    return m_pidController.atSetpoint();
  }
  public double getMeasurment(){
    return m_rackMotor.getEncoder().getPosition();
  }


  public static IntakeSubsystem getInstance(){
    if(m_instance == null){
      m_instance = new IntakeSubsystem();
    }
    return m_instance;
  }
}
