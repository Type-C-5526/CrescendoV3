package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.util.PIDUtil;

public class Constants {
    public static final double stickDeadband = 0.1;

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
