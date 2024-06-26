package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.math.PhoenixUnits;
import frc.robot.util.PIDUtil;

public class Constants {
    public static final double stickDeadband = 0.1;

    public static class Field {
        public static final double FIELD_WIDTH = 8.21;
        public static final double FIELD_LENGTH = 16.54;

        public static final Translation2d CENTER = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);

        public static final Translation2d BLUE_SPEAKER = new Translation2d(0.00,5.56 + 0.45);
        public static final Translation2d RED_SPEAKER = new Translation2d(16.54,5.15);
    }

    public static final class ShooterPivot {
        public static final int MOTOR_ID = 18;
        public static final String MOTOR_CANBUS = "rio";

        public static final int CANCODER_ID = 19;
        public static final String CANCODER_CANBUS = "rio";

        public static final PIDUtil PIVOT_PID_UTIL = new PIDUtil(150,0,0,PhoenixUnits.getDegreesToRotations(2));

        //public static final double MAGNET_OFFSET = -0.382080;
        //public static final double MAGNET_OFFSET = -0.762207;
        public static final double MAGNET_OFFSET = -0.347468;

        

       
    }

    public static final class Vision {

        public static boolean USING_VISION = true;

        /** Robot loop frequency */
        public static final int ROBOT_LOOP_HZ = 50;
        /** Robot loop period */
        public static final double ROBOT_LOOP_PERIOD = 1.0 / ROBOT_LOOP_HZ;

        public static final int NUMBER_OF_CAMERAS = 2;

        public static final String CAMERA_A_NAME = "Camera_Module_v1";

        public static final Transform3d CAMERA_A_LOCATION = new Transform3d(
            new Translation3d(0, Units.inchesToMeters(15), 0.187),
            new Rotation3d(0.0, Math.toRadians(-30.0), 0)
        );

        public static final Rotation2d CAMERA_A_FOV = Rotation2d.fromDegrees(78.44);

        public static final String CAMERA_B_NAME = "Arducam_OV9281_USB_Camera_B";

        public static final Transform3d CAMERA_B_LOCATION = new Transform3d(
            new Translation3d(0.00, Units.inchesToMeters(15), Units.inchesToMeters(7)),
            new Rotation3d(0.0, Math.toRadians(-25.0), 0.0)
        );

        public static final Rotation2d CAMERA_B_FOV = Rotation2d.fromDegrees(77.68);

    }

    public static final class IO {
        public static final int driverPort = 0;
    }

    public static final class Turret {
        public static final int MotorID = 6;
        public static final int MotorFollowerID = 7;

        public static final int MagneticSwitch1 = 0; 
        public static final int MagneticSwitch2 = 1; 


        public static final PIDUtil TurretPIDConstants = new PIDUtil(0.25,0,0,1);


    }


    public static final class Elevator {

        public static final int talonFXElevatorID = 3;
         public static final int talonFollowerFXElevatorID = 20;
                public static final String canbus = "rio";

                public static final TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

                public static final PIDUtil pidConstants = new PIDUtil(6, 0, 0, .1, 0.5);

    }


     public static final class Conveyor {
        public static final int MotorID = 23;
        public static final int ColorSensorChannel = 2;
    
        public static final PIDUtil ConveyorPIDConstants = new PIDUtil(0.145,0,0,1);


    }

     public static final class Intake {
        public static final int INTAKE_MOTOR_ID = 24;
        public static final int RACK_MOTOR_ID = 17;
         public static final PIDUtil TurretPIDConstants = new PIDUtil(0.45,0,0,1);


    }
    public static final class LEDs {
        public static final int LedPort = 0;
        public static final int LedLength = 18;
    }



    public static final class Shooter {
        public static final int talonFXShooterID = 1;
        public static final int talonFXIDCL = 31;
        public static final int talonFXICR = 32; //TODO Change ID 
        public static final String canbus = "rio";

        public static final TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

        public static final PIDUtil pidConstants = new PIDUtil(0.2, 0, 0, 10, 0.1);

    }
}
