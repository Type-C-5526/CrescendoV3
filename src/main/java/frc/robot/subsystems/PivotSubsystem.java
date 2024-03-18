// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivot;
import frc.robot.math.LinearInterpolation;
import frc.robot.math.PhoenixUnits;
import frc.robot.math.Point;

public class PivotSubsystem extends SubsystemBase {

  private static PivotSubsystem m_instance;

  private PIDController m_PID;
  private DutyCycleOut m_dutyCycle = new DutyCycleOut(0);

  private TalonFX m_motor;
  private CANcoder m_encoder;

  private boolean m_pidEnabled = false;
  private double m_setpoint = 0;

  private List<Point> m_points;

  private List<Point> m_points2;

    private List<Point> m_points3;

  private LinearInterpolation m_interpolation2;
  private LinearInterpolation m_interpolation;
  private LinearInterpolation m_interpolation3;


  private final SlewRateLimiter m_filter;
  private double m_forwardMaxOutput = 0.5;
  private double m_reverseMaxOutput = 0.2;



  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();


  

  /** Creates a new ShooterPivotSubsystem. */
  public PivotSubsystem() {
    m_points = new ArrayList<>();
    m_points2 = new ArrayList<>();
    m_points3 = new ArrayList<>();

    setTableValues();
    m_interpolation = new LinearInterpolation(m_points);
    m_interpolation2 = new LinearInterpolation(m_points2);
    m_interpolation3 = new LinearInterpolation(m_points3);




    m_filter = new SlewRateLimiter(0.01);
    


    m_PID = new PIDController(10.2, 0, 0);
    m_PID.setTolerance(0.05);
    

    m_motor = new TalonFX(ShooterPivot.MOTOR_ID, ShooterPivot.MOTOR_CANBUS);
    m_encoder = new CANcoder(ShooterPivot.CANCODER_ID, ShooterPivot.CANCODER_CANBUS);


    
    CANcoderConfiguration m_CANcoderConfiguration = new CANcoderConfiguration();
    
    //m_encoder.getConfigurator().apply(m_CANcoderConfiguration);
    
    m_CANcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    m_CANcoderConfiguration.MagnetSensor.MagnetOffset =  -0.777832;//-0.229492;
    m_CANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    
    m_encoder.getConfigurator().apply(m_CANcoderConfiguration);
    //m_encoder.getConfigurator().refresh(m_CANcoderConfiguration);  

    //m_encoder.setPosition(m_encoder.getAbsolutePosition().getValueAsDouble());
   
    TalonFXConfiguration m_FxConfiguration = new TalonFXConfiguration();
    m_FxConfiguration.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
    m_FxConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    m_FxConfiguration.Slot0.kP = ShooterPivot.PIVOT_PID_UTIL.getP(); // An error of 0.5 rotations results in 1.2 volts output
    m_FxConfiguration.Slot0.kD = ShooterPivot.PIVOT_PID_UTIL.getD(); // 0.1  A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    m_FxConfiguration.Voltage.PeakForwardVoltage = 12;
    m_FxConfiguration.Voltage.PeakReverseVoltage = -12;

    m_FxConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motor.getConfigurator().apply(m_FxConfiguration);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }




    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.


    var encoderPosition = m_encoder.getPosition().getValueAsDouble();
    var motorPosition = m_motor.getPosition().getValueAsDouble();


    SmartDashboard.putNumber("Pivot Absolut Position", m_encoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Cancoder Position", encoderPosition);
    SmartDashboard.putNumber("Pivot Talon Position", motorPosition);
    SmartDashboard.putNumber("Pivot Degrees", PhoenixUnits.getRotationsToDegrees(getMeasurment()));

    SmartDashboard.putBoolean("Pivot Enabled", m_pidEnabled);
    SmartDashboard.putBoolean("Pivot At Setpoint", atSetpoint());
    SmartDashboard.putNumber("Pivot Setpoint", m_setpoint);

    if (m_pidEnabled) {
      double output = m_PID.calculate(getMeasurment());

      SmartDashboard.putNumber("Pivot PID Output", output);
      SmartDashboard.putNumber("Slew Rate Pivot Output", m_filter.calculate(output));
      //output = m_filter.calculate(output);

      if(output > m_forwardMaxOutput){
        output = m_forwardMaxOutput;
      }else if(output < -m_reverseMaxOutput){
        output = -m_reverseMaxOutput;
      }

      m_motor.setControl(m_dutyCycle.withOutput(output));
      //m_motor.setControl(m_voltagePosition.withPosition(m_setpoint));
    }else{
      m_motor.setControl(m_brake);
    }
  }

  public void enablePID(){
    m_pidEnabled = true;
  }

  public void disablePID(){
    m_pidEnabled = false;
  }

  public void setSetpointInDegrees(double _setpoint){
    m_setpoint = PhoenixUnits.getDegreesToRotations(_setpoint);
    m_PID.setSetpoint(PhoenixUnits.getDegreesToRotations(_setpoint));
  }
  public double getSetpoint(){
    return m_setpoint;
  }
  public double getMeasurmentInDegrees(){
    return PhoenixUnits.getRotationsToDegrees(getMeasurment());
  }

  public boolean atSetpoint(){
    //return ShooterPivot.PIVOT_PID_UTIL.atSetpoint(m_encoder.getPosition().getValueAsDouble(), m_setpoint);
    return m_PID.atSetpoint();
  }

  public void resetPositionToAbsolute(){
    m_encoder.setPosition(m_encoder.getAbsolutePosition().getValueAsDouble());
  }

  public double getMeasurment(){
    return m_encoder.getAbsolutePosition().getValueAsDouble() + Constants.ShooterPivot.MAGNET_OFFSET;
  }

  public double getPivotTargetAngle(double _distance){
    return m_interpolation3.interpolate(_distance);
  }

  public boolean IsEnabled(){
    return m_pidEnabled;
  }
  private void setTableValues(){
    m_points.add(new Point(116.1,49.8));
    m_points.add(new Point(164.1, 44.4));
    m_points.add(new Point(194.6, 37.1));
    m_points.add(new Point(262.1, 33.8));
    m_points.add(new Point(267.6, 32.85));
    m_points.add(new Point(293.1, 31.00));
    m_points.add(new Point(329.1, 28.2));
    m_points.add(new Point(379.6, 26.8));
    m_points.add(new Point(460.1, 23.29));
    m_points.add(new Point(498.1, 21.58));
    m_points.add(new Point(554.1, 21.14));
    m_points.add(new Point(611.1, 20.26));

    m_points2.add(new Point(155.1, 46.878));
    m_points2.add(new Point(171.1, 41.044));
    m_points2.add(new Point(192.1, 40.165));
    m_points2.add(new Point(209.6, 37.089));
    m_points2.add(new Point(235.1, 35.244));
    m_points2.add(new Point(265.1, 31.816));
    m_points2.add(new Point(298.1, 29.970));
    m_points2.add(new Point(338.1, 26.191));
    m_points2.add(new Point(365.1, 25.521));
    m_points2.add(new Point(390.1, 23.994));
    m_points2.add(new Point(416.1, 22.939));
    m_points2.add(new Point(439.1, 22.1));
    m_points2.add(new Point(478.1, 20.214));
    m_points2.add(new Point(520.1, 19.599));
    m_points2.add(new Point(570.1, 19.863));

    m_points3.add(new Point(116.1,49.8));
    m_points3.add(new Point(164.1, 44.4));
    m_points3.add(new Point(194.6, 37.1));
    m_points3.add(new Point(262.1, 33.8));
    m_points3.add(new Point(267.6, 32.85));
    m_points3.add(new Point(298.1, 29.970));
    m_points3.add(new Point(338.1, 26.191));
    m_points3.add(new Point(365.1, 25.521));
    m_points3.add(new Point(390.1, 23.994));
    m_points3.add(new Point(416.1, 22.939));
    m_points3.add(new Point(439.1, 22.1));
    m_points3.add(new Point(478.1, 20.214));
    m_points3.add(new Point(520.1, 19.599));
    m_points3.add(new Point(570.1, 19.863));

        
    }



  public static PivotSubsystem getInstance(){
    if(m_instance == null){
      m_instance = new PivotSubsystem();
    }
    return m_instance;
  }

}
