// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.math.Vector;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotStatus;
import frc.robot.util.PoseHelper;

public class AutoAim extends Command {

  private Supplier<Pose2d> m_poseSupplier;
  private boolean isBlue;
  private PivotSubsystem m_Pivot;

  private ShooterSubsystem m_shooter;
  private ElevatorSubsystem m_elevator;
  
  
  private Timer m_timer;

  private Timer m_timer2;

  private double tolerance = 10.00;

  private boolean canAim;
  private boolean shooted;
  private boolean shootingBackwards;

  public boolean isC1;
  public boolean isC2;
  public boolean isC3;
  public boolean isC4;


  /** Creates a new AutoAim. */
  public AutoAim(Supplier<Pose2d> _poseSupplier) {
    m_poseSupplier = _poseSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();

    m_shooter = ShooterSubsystem.getInstance();
    m_elevator = ElevatorSubsystem.getInstance();  

    m_timer = new Timer();
    m_timer2 = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Superstructure.setRobotStatus(RobotStatus.AIMING);

    m_Pivot = PivotSubsystem.getInstance();
    m_Pivot.setSetpointInDegrees(0);
    m_Pivot.enablePID();

    m_shooter.setSetpoint(-90);
    m_shooter.enableMotorPID();

    shooted = false;
    
    shootingBackwards = false;

    m_timer.reset();
    m_timer.stop();

    isC1 = false;
    isC2 = false;
    isC3 = false;
    isC4 = false;

    m_timer.start();

    m_timer2.reset();
    m_timer2.stop();

    if (DriverStation.isAutonomous()) {
      IntakeSubsystem.getInstance().setSetpointAsPercent(0);
      IntakeSubsystem.getInstance().setSpeed(0);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    DriverStation.getAlliance().ifPresent((allianceColor) -> {
        if (allianceColor == Alliance.Blue) {
          isBlue = true;
        }else if (allianceColor == Alliance.Red) {
          isBlue = false; 
        }
    });

    Pose2d pose = m_poseSupplier.get();
    
    PoseHelper helper = new PoseHelper( 
      (isBlue) ? Field.BLUE_SPEAKER : Field.RED_SPEAKER, pose.getTranslation());
    double distance = helper.DistanceBetweenPoses();
    double angle = helper.AngleBetweenPoses();
    double heading = pose.getRotation().getDegrees();

    double pivotTargetAngle = m_Pivot.getPivotTargetAngle(distance * 100);

    SmartDashboard.putNumber("Pivot Angle Setpoint.....", m_Pivot.getPivotTargetAngle(distance * 100));
    SmartDashboard.putNumber("Robot Distance From Speaker", distance);

    if(heading < 0){
      heading += 360;
    }



    SmartDashboard.putNumber("Angle Antes de Arreglo", angle);

    //C1
    if(helper.DiffXBetweenPoses() <= 0 && helper.DiffYBetweenPoses() <= 0){

      
      angle = (90 - Math.abs(angle)) + 270;

      isC1 = true;
      isC2 = false;
      isC3 = false;
      isC4 = false; 
    }
    //C2
    else if(helper.DiffXBetweenPoses() >= 0 && helper.DiffYBetweenPoses() <= 0){

      isC1 = false;
      isC2 = true;
      isC3 = false;
      isC4 = false;

      angle = (90 - Math.abs(angle)) + 90;
    }
    //C3
    else if(helper.DiffXBetweenPoses() >= 0 && helper.DiffYBetweenPoses() >= 0){

      isC1 = false;
      isC2 = false;
      isC3 = true;
      isC4 = false;

      angle = Math.abs(angle) + 180;

    }
    //C4
    else if(helper.DiffXBetweenPoses() <= 0 && helper.DiffYBetweenPoses() >= 0){

      isC1 = false;
      isC2 = false;
      isC3 = false;
      isC4 = true;

      
      angle = Math.abs(angle);
    }

    SmartDashboard.putBoolean("IS in c1",  isC1 );    
    SmartDashboard.putBoolean("IS in c2",  isC2 );
    SmartDashboard.putBoolean("IS in c3",  isC3 );
    SmartDashboard.putBoolean("IS in c4",  isC4 );

    Vector vectorA = new Vector(1, heading, isBlue); //Is blue does nothing
    Vector vectorB = new Vector(distance, angle, isBlue);
    double angleBetweenVectors = Vector.getAngleBetweenVectors(vectorA, vectorB);

    double turretSetpoint = 0;

    int angleBetweenVectorsInt = (int) angleBetweenVectors;
    int vectorAAngle = ( int ) vectorA.getAngle();
    int vectorBAngle = ( int ) vectorB.getAngle();

    
    SmartDashboard.putNumber("Angle Between Vectors", angleBetweenVectors);
    SmartDashboard.putNumber("Heading", heading);
    SmartDashboard.putNumber("Angle Despues de Arreglo", angle);
    SmartDashboard.putNumber("Magnitud Speaker", distance);
    SmartDashboard.putNumber("Diff X", helper.DiffXBetweenPoses());
    SmartDashboard.putNumber("Diff Y", helper.DiffYBetweenPoses());

    SmartDashboard.putNumber("vector A Angle", vectorAAngle);
    SmartDashboard.putNumber("vector B Angle", vectorBAngle);

    int angleChecker = (Math.abs(vectorAAngle) + Math.abs(angleBetweenVectorsInt));

    if(angleChecker > 360){

      angleChecker = angleChecker - 360;
      
    }

    if(angleChecker - Math.abs(vectorBAngle) <= tolerance){//propuesta de para tener tolerance
      turretSetpoint = angleBetweenVectors;
    }
    else {
      turretSetpoint = angleBetweenVectors * -1;
    }


    /* 
    SmartDashboard.putNumber("Turret Setpoint VolteaDo", turretSetpoint);
    if(turretSetpoint > 90){

      shootingBackwards = true;
      if (isBlue) {
        turretSetpoint = turretSetpoint - 180;
        if (isC3) {
          turretSetpoint *= -1;
        }
      }
      
      m_elevator.setSetpointAsPercent(20);
      m_elevator.enableMotorPID();
      SmartDashboard.putNumber("Turret Setpoint VolteaDo FixeD", turretSetpoint);
      
    }
    else if(turretSetpoint < -90){

      shootingBackwards = true;

      if (isBlue) {
        turretSetpoint = turretSetpoint + 180;
        if (isC2) {
          turretSetpoint *= -1;
        }
      }

      m_elevator.setSetpointAsPercent(20);
      m_elevator.enableMotorPID();
      SmartDashboard.putNumber("Turret Setpoint VolteaDo FixeD", turretSetpoint);
    } else {
      shootingBackwards = false;
      m_elevator.setSetpointAsPercent(0);
      m_elevator.disableMotorPID();
    }*/

    /* 
    if(turretSetpoint > 90 || turretSetpoint < -90){
      turretSetpoint = 0;
      canAim = false;
    }
    else {
      canAim = true;
    }*/


    SmartDashboard.putNumber("Turret Supposed Setpoint: ", turretSetpoint);
    SmartDashboard.putBoolean("Is Blue:", isBlue);

    if(m_Pivot.getMeasurment() < 0){
      turretSetpoint = 0;
      canAim = false;
    }
    else {
      canAim= true;
    }

    if(pivotTargetAngle == Double.NaN){
      canAim = false;
    }
    else {
      canAim = true;

      m_Pivot.setSetpointInDegrees(pivotTargetAngle);
    }

    if (!DriverStation.isAutonomous()) {
      if(m_Pivot.atSetpoint() && m_shooter.atSetpoint() && Superstructure.isChasisAimed()){
        Superstructure.setRobotStatus(RobotStatus.AIMED);
        if(DriverStation.isAutonomous()){
          ConveyorBelt.getInstance().setMotorVelocity(1);
          shooted = true;
          m_timer2.start();
          
        }
      }
      else {
        Superstructure.setRobotStatus(RobotStatus.AIMING);
      }
    }else{
      if(m_Pivot.atSetpoint() && m_shooter.atSetpoint()){
        Superstructure.setRobotStatus(RobotStatus.AIMED);
        if(DriverStation.isAutonomous()){
          ConveyorBelt.getInstance().setMotorVelocity(1);
          shooted = true;
          m_timer2.start();
          
        }
      }
      else {
        Superstructure.setRobotStatus(RobotStatus.AIMING);
      }
    }

    
    

    
    //TurretSubsystem.getInstance().setSetpoint(TurretSubsystem.getAngleToTicks(turretSetpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!canAim){
      Superstructure.setRobotStatus(RobotStatus.CANT_AIM);
    }
    m_elevator.disableMotorPID();

    if(DriverStation.isAutonomous()){
      m_shooter.disableMotorPID();
    }

    new GoHome().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (shooted && m_timer2.get() > 0.8) {
      return true;
    }
    if(DriverStation.isAutonomous()){
      if (m_timer.get() > 3) {
        return true;
      }
    }
    return !canAim;
  }
}
