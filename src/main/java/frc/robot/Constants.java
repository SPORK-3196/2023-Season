package frc.robot;

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
        public static final int rearLeftPort = 1;
        public static final int frontLeftPort = 3; 
        public static final int rearRightPort = 2;
        public static final int frontRightPort = 4;

        public static final int gyroPort = 5;

        public static final int wheelRadiusIn = 3;
        public static final double wheelRadiusMeter = Units.inchesToMeters(wheelRadiusIn);
    }

    public static final class TurretConstants {
        public static final int turretPort = 9;

        public static final int motorVolts = 12;
        public static final double idealSpeed = 0; //robby help.
        public static final double motorRotationsPerDegree = 0.1745; //10:1 gear ratio
        public static final double motorRotationsPerDegree2022 = 0.1515; //8.68:1 gear ratio

    }
}
