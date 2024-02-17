package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.AprilTagCamera.Resolution;
import frc.robot.util.PIDUtil;

public class Constants {
    public static final double stickDeadband = 0.1;

    public static class Field {
        public static final double FIELD_WIDTH = 8.21;
        public static final double FIELD_LENGTH = 16.54;

        public static final Translation2d CENTER = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);
        public static final Pair<Integer,Translation2d> BLUE_SPEAKER = new Pair<Integer,Translation2d>(7, new Translation2d(0.00, 5.55));
        public static final Pair<Integer,Translation2d> RED_SPEAKER = new Pair<Integer,Translation2d>(4, new Translation2d(15.64, 5.55));

    }

    public static final class Vision {

        /** Robot loop frequency */
        public static final int ROBOT_LOOP_HZ = 50;
        /** Robot loop period */
        public static final double ROBOT_LOOP_PERIOD = 1.0 / ROBOT_LOOP_HZ;

        public static final int NUMBER_OF_CAMERAS = 1;

        public static final String CAMERA_A_NAME = "Arducam_OV9782_USB_Camera_A";

        public static final Transform3d CAMERA_A_LOCATION = new Transform3d(
            new Translation3d(0.381, 0.133, 0.102),
            new Rotation3d(0.0, Math.toRadians(-20.0), 0.0)
        );

        public static final Resolution CAMERA_A_RESOLUTION = Resolution.RES_1280_720;
        public static final Rotation2d CAMERA_A_FOV = Rotation2d.fromDegrees(79.7);

    }

    public static final class IO {
        public static final int driverPort = 0;
    }

    public static final class Turret {
        public static final int MotorID = 6;
        public static final int MotorFollowerID = 7;

        public static final int MagneticSwitch1 = 0; 
        public static final int MagneticSwitch2 = 1; 


        public static final PIDUtil TurretPIDConstants = new PIDUtil(0.145,0,0,1);


    }


    public static final class Shooter {
        public static final int talonFXShooterID = 34;
        public static final int talonFXIDCL = 31;
        public static final int talonFXICR = 32; //TODO Change ID 
        public static final String canbus = "rio";

        public static final TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

        public static final PIDUtil pidConstants = new PIDUtil(0.2, 0, 0, 10, 0.1);

    }
}
