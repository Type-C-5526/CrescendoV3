package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Field;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.Auto.NearShotBack;
import frc.robot.generated.TunerConstants;
import frc.robot.math.Vector;
import frc.robot.util.PoseHelper;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    boolean isBlue = false;

    public boolean isC1;
    public boolean isC2;
    public boolean isC3;
    public boolean isC4;

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this));
    private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SteerCharacterization.withVolts(volts)),
                    null,
                    this));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    private PIDController m_PIDHeading;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();

        isC1 = false;
        isC2 = false;
        isC3 = false;
        isC4 = false;

        //m_PIDHeading = new PIDController(0.02, 0, 0.001);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        m_PIDHeading = new PIDController(0.04, 0, 0.001);
        m_PIDHeading.setTolerance(0);

        isC1 = false;
        isC2 = false;
        isC3 = false;
        isC4 = false;

        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        NamedCommands.registerCommand("deployIntake", new DeployIntake());
        NamedCommands.registerCommand("autoAim", new AutoAim(() -> this.getState().Pose));
        NamedCommands.registerCommand("nearShotBack", new NearShotBack());
        NamedCommands.registerCommand("retractIntake", new RetractIntake());

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(4, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
            this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public DoubleSupplier getHeadingToApply(boolean isForAmp){
        return () -> {
            var alliance = DriverStation.getAlliance();
            double heading = this.getState().Pose.getRotation().getDegrees();

            if (heading < 0) {
                heading += 360;
            }

            Vector sourceVector;

           
            if(isForAmp){
                if(heading > 90 && heading <= 270){
                    sourceVector = new Vector(1, 180, true);
                }
                else {
                    sourceVector = new Vector(1, 0, true);
                }
            }
            else{
                if (alliance.isPresent()) {
                    if (alliance.get() == DriverStation.Alliance.Red) {
                        sourceVector = new Vector(1, 170, true);
                    }else{
                        sourceVector = new Vector(1, 340, true);
                    }
                }else{
                    sourceVector = new Vector(1, 0, true);
                }

            }

            

            SmartDashboard.putNumber("Robot Heading", heading);

            Vector headingVector = new Vector(1, heading, true); //Is blue does nothing
            double angleBetweenVectors = Vector.getAngleBetweenVectors(headingVector, sourceVector);

            SmartDashboard.putNumber("Angle Between Vectors: ", angleBetweenVectors);

            

            double differenceToApply;

            double differenceFromSourceTo0Degrees = 360 - sourceVector.getAngle();
            heading += differenceFromSourceTo0Degrees;

            if (heading > 360) {
                heading = heading - 360;
            }

            if(heading <= 180){
                differenceToApply = angleBetweenVectors;
            }else{
                differenceToApply = -angleBetweenVectors;
            }

            m_PIDHeading.setSetpoint(0);


            SmartDashboard.putNumber("Difference Applied: ", differenceToApply);
            
            return m_PIDHeading.calculate(differenceToApply);
        };
    }

    public DoubleSupplier aimToSpeaker(){
        return () -> {

            double heading = this.getState().Pose.getRotation().getDegrees();

            if (heading < 0) {
                heading += 360;
            }

            

            


            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                if (allianceColor == Alliance.Blue) {
                  isBlue = true;
                }else if (allianceColor == Alliance.Red) {
                  isBlue = false; 
                }
            });

            
            PoseHelper helper = new PoseHelper( 
            (isBlue) ? Field.BLUE_SPEAKER : Field.RED_SPEAKER, this.getState().Pose.getTranslation());
            double distance = helper.DistanceBetweenPoses();
            double angle = helper.AngleBetweenPoses();

            //C1
            if(helper.DiffXBetweenPoses() <= 0 && helper.DiffYBetweenPoses() <= 0){

            
                angle = Math.abs(angle);
        
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
        
                angle = (90 - Math.abs(angle)) + 270;
                
            }
  

            Vector sourceVector = new Vector(distance, angle, IsOnCANFD);

            SmartDashboard.putNumber("Robot Heading", heading);

            Vector headingVector = new Vector(1, heading, true); //Is blue does nothing
            double angleBetweenVectors = Vector.getAngleBetweenVectors(headingVector, sourceVector);

            SmartDashboard.putNumber("Angle Between Vectors: ", angleBetweenVectors);

            double differenceToApply;

            if (isBlue) {
                differenceToApply =  angleBetweenVectors;

                if (differenceToApply > 360) {
                    differenceToApply = differenceToApply - 360;
                }

                if((headingVector.getAngle() + differenceToApply) - sourceVector.getAngle()<= 5){
                    differenceToApply *= -1;
                }else{
                    differenceToApply *= 1;
                }


            }else{
                double differenceFromSourceTo0Degrees = 360 - sourceVector.getAngle();
                heading += differenceFromSourceTo0Degrees;

                if (heading > 360) {
                    heading = heading - 360;
                }

                if(heading <= 180){
                    differenceToApply = angleBetweenVectors;
                }else{
                    differenceToApply = -angleBetweenVectors;
                }

            }

            

            
            
            return m_PIDHeading.calculate(differenceToApply);
        };
    }

    @Override
    public void periodic() {

        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                  
                hasAppliedOperatorPerspective = true;
            });

            
        }
    }
}