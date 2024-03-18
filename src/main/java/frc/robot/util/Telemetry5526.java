// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/** Add your docs here. */
public class Telemetry5526 extends SubsystemBase{
    

    private ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();
    private PivotSubsystem m_pivot = PivotSubsystem.getInstance();
    private ConveyorBelt m_conveyor = ConveyorBelt.getInstance();
    private TurretSubsystem m_turret = TurretSubsystem.getInstance();
    private ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();

    private ShuffleboardTab m_SubsystemsTab = Shuffleboard.getTab("Subsystems Telemetry");

    private ShuffleboardLayout m_ShooterLayout = m_SubsystemsTab.getLayout("Shooter", BuiltInLayouts.kList)
    .withSize(2,5)
    .withPosition(0,0);

    private ShuffleboardLayout m_PivotLayout = m_SubsystemsTab.getLayout("Pivot", BuiltInLayouts.kList)
    .withSize(2,5)
    .withPosition(2,0);

    private ShuffleboardLayout m_ConveyorLayout = m_SubsystemsTab.getLayout("Conveyor", BuiltInLayouts.kList)
    .withSize(2,5)
    .withPosition(4,0);

     private ShuffleboardLayout m_TurretLayout = m_SubsystemsTab.getLayout("Turret", BuiltInLayouts.kList)
    .withSize(2,5)
    .withPosition(6,0);

       private ShuffleboardLayout m_ElevatorLayout = m_SubsystemsTab.getLayout("Elevator", BuiltInLayouts.kList)
    .withSize(2,5)
    .withPosition(8,0);

    private GenericEntry m_ShooterPIDIsEnabled = m_ShooterLayout.add("Shooter PID is Enabled:  ", false).getEntry();
    private GenericEntry m_ShooterIsAtSetpoint = m_ShooterLayout.add("Shooter Is At Setpoint: ", false).getEntry();
    private GenericEntry m_ShooterSetpoint = m_ShooterLayout.add("Shooter Setpoint: ", 0).getEntry();
    private GenericEntry m_ShooterVelocity = m_ShooterLayout.add("Shooter Velocity: ", 0).getEntry();
    private GenericEntry m_ShooterError = m_ShooterLayout.add("Shooter Error: ", 0).getEntry();

    private GenericEntry m_PivotEnabled = m_PivotLayout.add("Pivot Enabled: ", false).getEntry();    
    private GenericEntry m_PivotIsAtSetpoint = m_PivotLayout.add("Pivot At Setpoint: ", false).getEntry(); 
    private GenericEntry m_PivotSetpoint = m_PivotLayout.add("Pivot Setpoint: ", 0).getEntry();
    private GenericEntry m_PivotPositionEncoder = m_PivotLayout.add("Encoder Pivot Position: ", 0).getEntry();
    private GenericEntry m_PivotInDegrees = m_PivotLayout.add("Position In Degrees: ",0).getEntry();

    private GenericEntry m_ConveyorVelocity = m_ConveyorLayout.add("Conveyor Velocity: " , 0).getEntry();
    private GenericEntry m_ConveyorHasGamePiece = m_ConveyorLayout.add("Conveyor has Game Piece: " , false).getEntry();   
    private GenericEntry m_ConveyorIsHolding = m_ConveyorLayout.add("Conveyor Is Holding: " , false).getEntry();
    private GenericEntry m_ConveyorIsReleasing = m_ConveyorLayout.add("Conveyor is Releasing: " , false).getEntry();

    private GenericEntry m_TurretIsEnabled = m_TurretLayout.add("Turret Is Enabled: ", false).getEntry();   
    private GenericEntry m_TurretAtSetpoint = m_TurretLayout.add("Turret Is At Setpoint: ", false).getEntry();
    private GenericEntry m_TurretSetpoint = m_TurretLayout.add("Turret Setpoint: ", 0 ).getEntry();
    private GenericEntry m_TurretPosition = m_TurretLayout.add("Turret Position: ", 0 ).getEntry();
    private GenericEntry m_TurretAtHome = m_TurretLayout.add("Turret Is At Home: ", false).getEntry();

    private GenericEntry m_ElevatorIsEnabled = m_ElevatorLayout.add("Elevator Is Enabled: ",false).getEntry();
    private GenericEntry m_ElevatorAtSetpoint = m_ElevatorLayout.add("Elevator Is At Setpoint: ",false).getEntry();
    private GenericEntry m_ElevatorSetpoint = m_ElevatorLayout.add("Elevator Setpoint: ",0).getEntry();
    private GenericEntry m_ElevatorPosition = m_ElevatorLayout.add("Elevator Position: ",0).getEntry();




    @Override
    public void periodic(){
        updateLayout();
    }

    public void updateLayout(){
        m_ShooterVelocity.setDouble(m_shooter.getShooterVelocity());
        m_ShooterError.setDouble(m_shooter.getError());
        m_ShooterSetpoint.setDouble(m_shooter.getSetpoint());
        m_ShooterIsAtSetpoint.setBoolean(m_shooter.atSetpoint());
        m_ShooterPIDIsEnabled.setBoolean(m_shooter.isEnabled());

        m_PivotSetpoint.setDouble(m_pivot.getSetpoint());
        m_PivotPositionEncoder.setDouble(m_pivot.getMeasurment());
        m_PivotEnabled.setBoolean(m_pivot.IsEnabled());
        m_PivotIsAtSetpoint.setBoolean(m_pivot.atSetpoint());
        m_PivotInDegrees.setDouble(m_pivot.getMeasurmentInDegrees());

        m_ConveyorVelocity.setDouble(m_conveyor.getVelocity());
        m_ConveyorIsHolding.setBoolean(m_conveyor.isHolding());
        m_ConveyorIsReleasing.setBoolean(m_conveyor.isReleasing());
        m_ConveyorHasGamePiece.setBoolean(m_conveyor.hasGamePiece());

        m_TurretSetpoint.setDouble(m_turret.getSetpoint());
        m_TurretAtHome.setBoolean(m_turret.isHome());
        m_TurretAtSetpoint.setBoolean(m_turret.isAtSetpoint());
        m_TurretPosition.setDouble(m_turret.getMeasurment());
        m_TurretIsEnabled.setBoolean(m_turret.isEnabled());

        m_ElevatorAtSetpoint.setBoolean(m_elevator.atSetpoint());
        m_ElevatorIsEnabled.setBoolean(m_elevator.isEnabled());
        m_ElevatorPosition.setDouble(m_elevator.getPosition());
        m_ElevatorSetpoint.setDouble(m_elevator.getSetpoint());









        


    }



}
