// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.Superstructure.RobotStatus;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new Turret. */
  private static TurretSubsystem m_instance;

  private CANSparkMax m_motor1;
  private CANSparkMax m_followerMotor2;

  private DigitalInput m_magneticSwitch1;
  private DigitalInput m_magneticSwitch2;

  private RelativeEncoder m_encoder;

  private double m_setpoint = 0.0;
  private boolean m_enabled;

  private PIDController m_pidController;

  private double m_maxOutput = 0.5;
  private SlewRateLimiter m_filter;


  public TurretSubsystem() {

    m_enabled = false;

    m_magneticSwitch1 = new DigitalInput(Constants.Turret.MagneticSwitch1);
    m_magneticSwitch2 = new DigitalInput(Constants.Turret.MagneticSwitch2);

    m_pidController = new PIDController(Turret.TurretPIDConstants.getP(), Turret.TurretPIDConstants.getI(), Turret.TurretPIDConstants.getD());
    m_pidController.setTolerance(0.01);
  
    m_filter = new SlewRateLimiter(10);

    m_motor1 = new CANSparkMax(Constants.Turret.MotorID, MotorType.kBrushless);
    m_followerMotor2 = new CANSparkMax(Constants.Turret.MotorFollowerID, MotorType.kBrushless);

    m_motor1.restoreFactoryDefaults();
    m_followerMotor2.restoreFactoryDefaults();

    m_motor1.setSmartCurrentLimit(30);
    m_followerMotor2.setSmartCurrentLimit(30);
    m_motor1.setIdleMode(IdleMode.kBrake);



  


    m_encoder = m_motor1.getEncoder();
    m_encoder.setPosition(0.0);



    m_followerMotor2.follow(m_motor1);
    
    
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double output = m_pidController.calculate(getMeasurment());
    output = m_filter.calculate(output);

    if (Superstructure.getRobotStatus() == RobotStatus.SCORING_IN_AMP) {
      m_maxOutput = 0.3;
    }else{
      m_maxOutput = 0.5;
    }

    SmartDashboard.putNumber("Turret Value: ", m_motor1.getEncoder().getPosition());
    SmartDashboard.putNumber("Turret Angle: ", getConvertedAngle(m_motor1.getEncoder().getPosition()));
    SmartDashboard.putBoolean("Is Home: ", isHome());
    SmartDashboard.putNumber("Turret Output", output);
    SmartDashboard.putBoolean("Turret Is Enabled: ", m_enabled);


    if(m_enabled){

      SmartDashboard.putNumber("Turret Output: ", output);
      if(output > m_maxOutput){
        output = m_maxOutput;
      }else if(output < -m_maxOutput){
        output = - m_maxOutput;
      }
      m_motor1.set(output);
    }
    else {
      m_motor1.stopMotor();
    }
  }

  public void setSetpoint(double _Setpoint){
    m_setpoint = _Setpoint;
    m_pidController.setSetpoint(_Setpoint);
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

  public double getMeasurment(){
    return m_motor1.getEncoder().getPosition();
  }
  public double getSetpoint(){
    return m_setpoint;
  }
  public boolean isEnabled(){
    return m_enabled;
  }



  public static TurretSubsystem getInstance(){
    if(m_instance == null){
      m_instance = new TurretSubsystem();
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
