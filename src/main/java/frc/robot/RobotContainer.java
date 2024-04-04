// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.FeedFromSource;
import frc.robot.commands.FeedNotes;
import frc.robot.commands.LeaveAmp;
import frc.robot.commands.NearShot;
import frc.robot.commands.OpenAll;
import frc.robot.commands.SafeZoneShot;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  
  private ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private LEDSubsystem m_ledSubsystem = LEDSubsystem.getInstance();
  private ConveyorBelt m_conveyorBelt = ConveyorBelt.getInstance();
  private IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  private VisionSubsystem m_vision = VisionSubsystem.getInstance();

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final XboxController driver_HID = driver.getHID();

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final CommandXboxController operator = new CommandXboxController(1); 

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.10).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.FieldCentric driveWithoutRotationalDeadband = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private Command nearGrab3 = drivetrain.getAutoPath("Auto2");
  private Command swipeNotes = drivetrain.getAutoPath("Auto3");
  private Command shotAndLeave = drivetrain.getAutoPath("Auto4");

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final PivotSubsystem m_pivot = PivotSubsystem.getInstance();


  private void configureBindings() {

    

    //TurretSubsystem.getInstance().setDefaultCommand(new AutoAim(() -> drivetrain.getState().Pose));
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver_HID.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver_HID.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver_HID.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

        
   driver.a().whileTrue(drivetrain.applyRequest(() -> brake)); 
   


     new Trigger(() -> driver_HID.getLeftTriggerAxis() > 0.5).whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-(driver_HID.getLeftY() * MaxSpeed) / 5) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY((-driver_HID.getLeftX() * MaxSpeed) / 5) // Drive left with negative X (left)
            .withRotationalRate((-driver_HID.getRightX() * MaxAngularRate) / 5) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    driver.x().whileTrue(
      drivetrain.applyRequest(() -> driveWithoutRotationalDeadband.withVelocityX(-driver_HID.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver_HID.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.getHeadingToApply(false).getAsDouble()) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));
    
    

    // reset the field-centric heading on left bumper press

    drivetrain.registerTelemetry(logger::telemeterize);
    

    driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
 
    //drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.36,5.56), new Rotation2d(180)));  //TODO: Remove Initial Position

    driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));


    //driver.a().whileTrue(new LeaveAmp());
    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
/* 
    driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
*/
    operator.leftBumper().whileTrue(new DeployIntake());
    //operator.leftBumper().onTrue(new InstantCommand(() -> m_intake.setSpeed(1)));

    //driver.a().whileTrue(new ShootTest());
    //driver.y().whileTrue(new ConveyorIn());
     
    operator.rightBumper().whileTrue(new Shoot());   
    operator.y().whileTrue(new FeedFromSource());
    operator.a().whileTrue(new AutoAim(() -> drivetrain.getState().Pose)); 
    operator.a().whileTrue(
      drivetrain.applyRequest(() -> driveWithoutRotationalDeadband.withVelocityX(-driver_HID.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver_HID.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.aimToSpeaker().getAsDouble()) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));
    operator.x().whileTrue(new LeaveAmp(() -> drivetrain.getState().Pose));
    
    /* 
    operator.x().whileTrue(
      drivetrain.applyRequest(() -> driveWithoutRotationalDeadband.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.getHeadingToApply(true).getAsDouble()) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));*/

    operator.b().onTrue(new InstantCommand(() -> ConveyorBelt.getInstance().setMotorVelocity(-1)));
    operator.b().onFalse(new InstantCommand(() -> ConveyorBelt.getInstance().setMotorVelocity(-0.2)));

    operator.povUp().whileTrue(new SafeZoneShot());
    operator.povDown().whileTrue(new NearShot());

    operator.leftStick().whileTrue(new OpenAll());

    new Trigger(() -> operator.getRightTriggerAxis() > 0.5).whileTrue(new FeedNotes());

    operator.back().onTrue(new InstantCommand(() -> IntakeSubsystem.getInstance().setSetpointAsPercent(100)));

    operator.povLeft().onTrue(new InstantCommand(() -> Superstructure.switchIgnoreColorSensor()));
    operator.povRight().onTrue(new InstantCommand(() -> Superstructure.switchIgnoreAimed()));
  }

  public RobotContainer() {
    
    configureBindings();

    m_chooser.setDefaultOption("Default Auto", null);
    m_chooser.addOption("Near, Grab 3", nearGrab3);
    m_chooser.addOption("Swipe Notes", swipeNotes);
    m_chooser.addOption("Shot And Leave", shotAndLeave);
    SmartDashboard.putData("Auto choices", m_chooser);

    //m_Limelight = new Limelight(drivetrain, "limelight-a");
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return m_chooser.getSelected();
  }
}