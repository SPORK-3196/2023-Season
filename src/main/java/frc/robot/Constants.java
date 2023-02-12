package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class ClawConstants {
        public static final int piston1Port = 10;
        public static final int piston2Port = 11;

        public static final int kSolenoidButton = 1;
        public static final boolean kSolenoidForward = true;
        public static final boolean kSolenoidReverse = false;
    }

    public static final class ArmConstants {
        public static final int elbowPort = 6;
        public static final int shoulderPort = 7;
        
        public static final double kP = 1;

        public static final double kSVolts = 1;
        public static final double kGVolts = 1;
        public static final double kVVoltSecondPerRad = 0.5;
        public static final double kAVoltSecondSquaredPerRad = 0.1;

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
        public static double m_2022kP = 2.7381;
        
        public static double m_2022DrivetrainTrackWidthMeters = Units.inchesToMeters(28);
        public static DifferentialDriveKinematics m_2022DifferentialDriveKinematics = new DifferentialDriveKinematics(m_2022DrivetrainTrackWidthMeters);

        public static double maxSpeed = 4;
        public static double kMaxAccelerationMetersPerSecSquared = 3;

    }
}
