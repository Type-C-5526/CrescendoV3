// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private TalonFX m_motor1;
  private TalonFX m_followerMotor;

  private TalonFXConfiguration m_motor1Configuration;
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  private double m_setpoint;

  private boolean m_enabled;

  private NeutralOut m_break;

  private static ElevatorSubsystem m_instance;
  


  public ElevatorSubsystem() {

    m_setpoint = 0;
    m_enabled = false;
    m_break = new NeutralOut();
    m_motor1 = new TalonFX(Constants.Elevator.talonFXElevatorID, Constants.Elevator.canbus);
    m_followerMotor = new TalonFX(Constants.Elevator.talonFollowerFXElevatorID, Constants.Elevator.canbus);

    m_motor1Configuration = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs mm = m_motor1Configuration.MotionMagic;
    mm.MotionMagicCruiseVelocity = 40; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 50; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    mm.MotionMagicJerk = 90;

    Slot0Configs slot0 = m_motor1Configuration.Slot0;
    slot0.kP = 50;
    slot0.kI = 0;
    slot0.kD = 0.005;
    slot0.kV = 0.22;
    slot0.kS = 2.0; // Approximately 0.25V to get the mechanism moving
    
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = m_motor1.getConfigurator().apply(m_motor1Configuration);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    m_followerMotor.getConfigurator().apply(m_motor1Configuration);

    m_followerMotor.setControl(new Follower(m_motor1.getDeviceID(), true));
    m_motor1.setPosition(0);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_enabled){
      m_motor1.setControl(m_mmReq.withPosition(m_setpoint).withSlot(0));
    }
    else {
      m_motor1.setControl(m_break);
    }
    SmartDashboard.putNumber("Elevator encoder motor", m_motor1.getPosition().getValueAsDouble());

  }

  public void setSetpointAsPercent(double _Setpoint){
    m_setpoint = _Setpoint * -0.0977055;
  }

  public void enableMotorPID() {
    m_enabled = true;
  }

  public void disableMotorPID() {
    m_enabled = false;
  }

  public boolean atSetpoint(){
    return Constants.Elevator.pidConstants.atSetpoint(m_motor1.getPosition().getValueAsDouble(), m_setpoint);
  }

  public boolean isEnabled(){
    return m_enabled;
  }

  public double getSetpoint(){
    return m_setpoint;
  }
  public double getPosition(){
    return m_motor1.getPosition().getValueAsDouble();
  }





  public static ElevatorSubsystem getInstance(){
    if(m_instance == null){
      m_instance = new ElevatorSubsystem();
    }
    return m_instance;
  }

}
