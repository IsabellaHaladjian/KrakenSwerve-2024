package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    /*============================
               Swerve 
    ==============================*/

        /* Gyro Constants */
        public static final int PIGEON_ID = 19;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        /* Chosen Module */
        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDS_MK4i_L5);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(20.75);
        public static final double WHEEL_CIRCUMFERENCE = chosenModule.wheelCircumference;

        /* Swerve Kinematics */
         public static final Translation2d[] moduleTranslations = new Translation2d[]{
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)};

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(moduleTranslations);

        public static final double DRIVETRAIN_RADIUS = Units.inchesToMeters(14.67247);

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = chosenModule.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_INVERT = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int AZIMUTH_CURRENT_LIMIT = 25;
        public static final int AZIMUTH_CURRENT_THRESHOLD = 40;
        public static final double AZIMUTH_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean AZIMUTH_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 35;   
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double AZIMUTH_P = chosenModule.angleKP;
        public static final double AZIMUTH_I = chosenModule.angleKI;
        public static final double AZIMUTH_D = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.12;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_S = 0.32;
        public static final double DRIVE_V = 1.51;
        public static final double DRIVE_A = 0.27;

        public static final double RATE_LIMITER = 1.5;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = Units.feetToMeters(23.25);
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = Math.PI * 4.12 * 0.5;

        /* Neutral Modes */
        public static final NeutralModeValue AZIMUTH_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-11.77);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(146.43);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 17;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(134.73);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(122.87);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    /*============================
                Auton
    ==============================*/

        /* Auto Constraints */
        public static final double AUTO_MAX_SPEED_MPS = 3;
        public static final double AUTO_MAX_ACCELERATION_MPS_SQUARED = 3;
        public static final double AUTO_ANGULAR_SPEED = Math.PI;
        public static final double AUTO_ANGULAR_ACCELERATION = Math.PI * Math.PI;
    
        /* Auto PID Constants */
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Auto Side Offsets */
        public static final double AUTO_AMP_SIDE_START_OFFSET = 64.5; //todo adjust this
        public static final double AUTO_SOURCE_SIDE_START_OFFSET = -64.5; //todo adjust this
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                AUTO_ANGULAR_SPEED, AUTO_ANGULAR_ACCELERATION);

        /* Pathplanner Auton Config */
        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(kPXController, 0, 0), // Translation constants 
            new PIDConstants(kPYController, 0, 0), // Rotation constants 
            MAX_SPEED, 
            Constants.DRIVETRAIN_RADIUS, // Drive base radius (distance from center to furthest module) 
            new ReplanningConfig()
        );

    /*============================
         Controller Constants
    ==============================*/

    public static final double stickDeadband = 0.1;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;


}
