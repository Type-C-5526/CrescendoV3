// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private static Turret m_instance;

  private CANSparkMax m_motor1;
  private CANSparkMax m_followerMotor2;

  private DigitalInput m_magneticSwitch1;
  private DigitalInput m_magneticSwitch2;

  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;

  private double m_setpoint = 0.0;
  private boolean m_enabled;


  public Turret() {

    m_enabled = false;

    m_magneticSwitch1 = new DigitalInput(Constants.Turret.MagneticSwitch1);
    m_magneticSwitch2 = new DigitalInput(Constants.Turret.MagneticSwitch2);

    m_motor1 = new CANSparkMax(Constants.Turret.MotorID, MotorType.kBrushless);
    m_followerMotor2 = new CANSparkMax(Constants.Turret.MotorFollowerID, MotorType.kBrushless);

    m_motor1.restoreFactoryDefaults();
    m_followerMotor2.restoreFactoryDefaults();

    m_pidController = m_motor1.getPIDController();

    m_encoder = m_motor1.getEncoder();
    m_encoder.setPosition(0.0);

    m_pidController.setP(Constants.Turret.TurretPIDConstants.getP());
    m_pidController.setI(Constants.Turret.TurretPIDConstants.getI());
    m_pidController.setD(Constants.Turret.TurretPIDConstants.getD());
    m_pidController.setOutputRange(-1, 1);

    m_followerMotor2.follow(m_motor1);

    
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Turret Value: ", m_motor1.getEncoder().getPosition());
    SmartDashboard.putNumber("Turret Angle: ", getConvertedAngle(m_motor1.getEncoder().getPosition()));
    if(m_enabled){
      m_pidController.setReference(m_setpoint, CANSparkMax.ControlType.kPosition);
    }
    else {
      m_motor1.stopMotor();
    }
  }

  public void setSetpoint(double _Setpoint){
    m_setpoint = _Setpoint;
  }

  public void setMotorVelocity(double _velocity){
    m_motor1.set(_velocity);
  }

  public void enableTurretPID(){
    m_enabled = true;
  }

   public void disableTurretPID(){
    m_enabled = false;
  }

  public void stopTurret(){
    m_motor1.stopMotor();
  }

  public void resetEncoder(){
    m_encoder.setPosition(0.0);
  }


  public static Turret getInstance(){
    if(m_instance == null){
      m_instance = new Turret();
    }
    return m_instance;
  }

  public double getConvertedAngle(double _encoderTicks){

    return (_encoderTicks * (180/36.166325));

  }


  public static double getAngleToTicks(double _angle){

    return _angle * (36.166325/180); 
  }

  public boolean getMagneticSwitch1(){
    return !m_magneticSwitch1.get();
  }

   public boolean getMagneticSwitch2(){
    return !m_magneticSwitch2.get();
  }

  public boolean isHome(){
    return getMagneticSwitch1() && getMagneticSwitch2();
  }

  public boolean isAtSetpoint(){
    return Constants.Turret.TurretPIDConstants.atSetpoint(m_motor1.getEncoder().getPosition(), m_setpoint);
  }

}
