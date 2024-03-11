// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.ShooterPivot;
import frc.robot.math.Vector;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Superstructure.RobotStatus;
import frc.robot.util.PoseHelper;

public class AutoAim extends Command {

  private Supplier<Pose2d> m_poseSupplier;
  private boolean isBlue;
  private PivotSubsystem m_Pivot;

  private ShooterSubsystem m_shooter;
  private TurretSubsystem m_turret;
  
  


  private double tolerance = 5.00;

  private boolean canAim;
  /** Creates a new AutoAim. */
  public AutoAim(Supplier<Pose2d> _poseSupplier) {
    m_poseSupplier = _poseSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(TurretSubsystem.getInstance());

    m_shooter = ShooterSubsystem.getInstance();
    m_turret = TurretSubsystem.getInstance();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Superstructure.setRobotStatus(RobotStatus.AIMING);

    m_Pivot = PivotSubsystem.getInstance();
    m_Pivot.setSetpointInDegrees(0);
    m_Pivot.enablePID();

    m_shooter.setSetpoint(-80);
    m_shooter.enableMotorPID();

    m_turret.setSetpoint(0);
    m_turret.enableTurretPID();

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

      angle = Math.abs(angle);
    }
    //C2
    else if(helper.DiffXBetweenPoses() >= 0 && helper.DiffYBetweenPoses() <= 0){

      angle = (90 - Math.abs(angle)) + 90;
    }
    //C3
    else if(helper.DiffXBetweenPoses() >= 0 && helper.DiffYBetweenPoses() >= 0){
      angle = Math.abs(angle) + 180;

    }
    //C4
    else if(helper.DiffXBetweenPoses() <= 0 && helper.DiffYBetweenPoses() >= 0){

      angle = (90 - Math.abs(angle)) + 270;
    }

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

    

    if((Math.abs(vectorAAngle) + Math.abs(angleBetweenVectorsInt)) - Math.abs(vectorBAngle) <= tolerance){//propuesta de para tener tolerance
      turretSetpoint = angleBetweenVectors;
    }
    else {
      turretSetpoint = angleBetweenVectors * -1;
    }

    if(turretSetpoint > 90 || turretSetpoint < -90){
      turretSetpoint = 0;
      canAim = false;
    }
    else {
      canAim = true;
    }

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

    if(m_Pivot.atSetpoint() && m_shooter.atSetpoint() && m_turret.isAtSetpoint() && canAim){
      Superstructure.setRobotStatus(RobotStatus.AIMED);
    }
    else {
      Superstructure.setRobotStatus(RobotStatus.AIMING);
    }

    
    TurretSubsystem.getInstance().setSetpoint(TurretSubsystem.getAngleToTicks(turretSetpoint));
    

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new GoHome().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
