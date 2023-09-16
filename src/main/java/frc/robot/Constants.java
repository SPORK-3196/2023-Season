package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class ClawConstants {

        public static final int LeftclawMotorPort = 10;
        public static final int RightClawMotorPort = 6;
        
    }

    public static final class ArmConstants {
        public static final int elbowPort = 8;
        public static final int shoulderPort = 7;
        
        public static final double kP = 1;

        public static final double restShoulderTick = -0;
        public static final double restElbowTick = 0;
        public static final double lowShoulderTick = -6;
        public static final double lowElbowTick = 34;
        public static final double midCubeShoulderTick = -16;
        public static final double midCubeElbowTick = 0;
        public static final double midConeShoulderTick = 0; // no
        public static final double midConeElbowTick = 0;// no
        public static final double highCubeShoulderTick = -23; 
        public static final double highCubeElbowTick = 37;
        public static final double highConeElbowTick = 0;// no
        public static final double highConeShoulderTick = 0;// no
        public static final double pickUpShoulderTick = 0;
        public static final double pickUpElbowTick = 0;
        public static final int pickUpStationShoulderTick = 0;// no
        public static final double pickUpStationElbowTick = 0;// no
        public static final int startElbowTick = 0;
        public static final double startShoulderTick = 0;

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

        public static double m_2022ksVolts = 0.19128;
        public static double m_2022kvVoltSecondsPerMeter = 5.2133;
        public static double m_2022kaVoltSecondsSquaredPerMeter = 1.2024;
        public static double m_2022kP = 2.651; //1.7381;
        
        public static double m_2022DrivetrainTrackWidthMeters = Units.inchesToMeters(26);
        public static DifferentialDriveKinematics m_2022DifferentialDriveKinematics = new DifferentialDriveKinematics(m_2022DrivetrainTrackWidthMeters);

        public static double maxSpeed = .5;
        public static double kMaxAccelerationMetersPerSecSquared = .1;

    }

    public static final class LiftConstants {
        public static final int liftMotorID = 8;

        public static final double liftKP = .5;
        public static final double liftKD = 0;
        public static final double liftKI = .0005;
        public static final double liftKFF = 1;

        public static final double liftRestTick = -25.7;
        public static final double liftBottomTick = -20;
        public static final double liftMidTick = -7.5;
        public static final double liftTopTick = -7.5;
        public static final double liftPickUpTick = -25.7;
        // public static final int liftStationTick = 4000;
        public static final double liftStartTick = -25;
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