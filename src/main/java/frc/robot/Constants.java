package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class ClawConstants {
        public static final int piston1Port = 10;
        public static final int piston2Port = 11;

        public static final int clawMotorPort = 13;
        
        public static final int kSolenoidButton = 1;
        public static final boolean kSolenoidForward = true;
        public static final boolean kSolenoidReverse = false;
    }

    public static final class ArmConstants {
        public static final int elbowPort = 6;
        public static final int shoulderPort = 7;
        
        public static final double kP = 1;

        public static final double shoulderKsVolts = 1;
        public static final double shoulderKgVolts = 1;
        public static final double shoulderKvVoltSecondPerRad = 0.5;
        public static final double shoulderKaVoltSecondSquaredPerRad = 0.1;

        public static final double elbowKsVolts = 1;
        public static final double elbowKgVolts = 1;
        public static final double elbowKvVoltSecondPerRad = 0.5;
        public static final double elbowKaVoltSecondSquaredPerRad = 0.1;

        public static final double kMaxVelocityRadPerSecond = 3;
        public static final double kMaxAccelerationRadPerSecSquared = 1;

        public static final int kEncoderPPR = 256;
        public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;

        public static final double kArmOffsetRads = 0.5;
    }
    
    public static final class DrivetrainConstants {
        public static final int rearLeftPort = 3;
        public static final int frontLeftPort = 1; 
        public static final int rearRightPort = 4;
        public static final int frontRightPort = 2;

        public static final int gyroPort = 12;

        public static final double wheelRadiusMeter = Units.inchesToMeters(3);
        public static final double m_2023gearRatio = 11.7;
        public static final double m_2022gearRatio = 8.68;
        public static final double countsPerRevolution = 2048;

        public static double m_2022ksVolts = 0.67962;
        public static double m_2022kvVoltSecondsPerMeter = 1.8334;
        public static double m_2022kaVoltSecondsSquaredPerMeter = 0.45089;
        public static double m_2022kP = .5; //1.7381;
        
        public static double m_2022DrivetrainTrackWidthMeters = Units.inchesToMeters(26);
        public static DifferentialDriveKinematics m_2022DifferentialDriveKinematics = new DifferentialDriveKinematics(m_2022DrivetrainTrackWidthMeters);

        public static double maxSpeed = .5;
        public static double kMaxAccelerationMetersPerSecSquared = .1;

    }

    public static final class LiftConstants {
        public static int liftMotorID = 14;

        public static double liftKP = 0.014;
        public static double liftKD = 0;
        public static double liftKI = 0;
    }
    public static final class VisionConstants {
        public static double globalShutterHeightMeters = Units.inchesToMeters(33.5);
        public static double aprilTagHeightMeters = Units.inchesToMeters(15.25);
        //  public static double aprilTagHeightMeters = Units.inchesToMeters(21.125);

        public static double globalShutterPitch = 0;
    }
}