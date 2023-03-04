package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class ClawConstants {

        public static final int clawMotorPort = 10;
        
    }

    public static final class ArmConstants {
        public static final int elbowPort = 6;
        public static final int shoulderPort = 7;
        
        public static final double kP = 1;

        public static final int restShoulderTick = 0;
        public static final int restElbowTick = 0;
        public static final int lowShoulderTick = 0;
        public static final int lowElbowTick = 0;
        public static final int midShoulderTick = 0;
        public static final int midElbowTick = 0;
        public static final int highShoulderTick = 0;
        public static final int highElbowTick = 0;
        public static final int pickUpShoulderTick = 0;
        public static final int pickUpElbowTick = 0;
        public static final int pickUpStationShoulderTick = 0;
        public static final int pickUpStationElbowTick = 0;
        public static final int startElbowTick = -500 ;
        public static final int startShoulderTick = 16300;

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

        public static final int gyroPort = 11;

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
        public static final int liftMotorID = 8;

        public static final double liftKP = 0.014;
        public static final double liftKD = 0;
        public static final double liftKI = 0;

        public static final int liftRestTick = 0;
        public static final int liftBottomTick = 500;
        public static final int liftMidTick = 3500;
        public static final int liftTopTick = 7000;
        public static final int liftPickUpTick = 450;
        public static final int liftStationTick = 4000;
        public static final int liftStartTick = -15500;
    }
    public static final class VisionConstants {
        public static double globalShutterHeightMeters = Units.inchesToMeters(33.5);
        public static double aprilTagHeightMeters = Units.inchesToMeters(15.25);

        public static double globalShutterPitch = 0;
    }

    public static final class TurretConstants {
        public static final int turretPort = 9;

        public static final double idealSpeed = 0; //robby help.
        public static final double motorRotationsPerDegree2023 = 0.1515; //8.68:1 gear ratio
    }
}
