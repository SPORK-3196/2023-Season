package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
<<<<<<< HEAD

=======
>>>>>>> 95f7bed121063f9b3a4af131fe5aa0ee1feb6a38

public class Drivetrain extends SubsystemBase {

    public WPI_TalonFX rearLeft = new WPI_TalonFX(DrivetrainConstants.rearLeftPort);
    public WPI_TalonFX frontLeft = new WPI_TalonFX(DrivetrainConstants.frontLeftPort);
    public WPI_TalonFX rearRight = new WPI_TalonFX(DrivetrainConstants.rearRightPort);
    public WPI_TalonFX frontRight = new WPI_TalonFX(DrivetrainConstants.frontRightPort);

    private WPI_PigeonIMU gyroscope = new WPI_PigeonIMU(DrivetrainConstants.gyroPort);

    public MotorControllerGroup leftGroup = new MotorControllerGroup(frontLeft, rearLeft);
    public MotorControllerGroup rightGroup = new MotorControllerGroup(frontRight, rearRight);

    public DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    
    public DifferentialDriveOdometry m_odometry;
    
    public Drivetrain() {
        rightGroup.setInverted(true);

        frontLeft.setSelectedSensorPosition(0);
        rearLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
        rearRight.setSelectedSensorPosition(0);
        gyroscope.setYaw(0);

        frontLeft.setNeutralMode(NeutralMode.Coast);
        frontRight.setNeutralMode(NeutralMode.Coast);
        rearLeft.setNeutralMode(NeutralMode.Coast);
        rearRight.setNeutralMode(NeutralMode.Coast);

        m_odometry = new DifferentialDriveOdometry(gyroscope.getRotation2d(), sensorToMeters(rearLeft.getSelectedSensorPosition()), -1 * sensorToMeters(rearRight.getSelectedSensorPosition()));
    }
    public void resetEncoders() {
        frontLeft.setSelectedSensorPosition(0);
        rearLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
        rearRight.setSelectedSensorPosition(0);
    }
    
    public void arcadeDrive(double speed, double rot){
        differentialDrive.arcadeDrive(speed, rot);
    }
    
    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts){
        leftGroup.setVoltage(leftVolts);
        rightGroup.setVoltage(rightVolts);
    }

    public double sensorToMeters(double sensorCount){
        double sensorRotations = sensorCount / Constants.DrivetrainConstants.countsPerRevolution;
        double motorRotations = sensorRotations / Constants.DrivetrainConstants.m_2022gearRatio;
         return motorRotations * (2 * Math.PI * Constants.DrivetrainConstants.wheelRadiusMeter);
    }
    
    public DifferentialDriveWheelSpeeds motorWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(
            sensorToMeters(rearLeft.getSelectedSensorVelocity()),
            -1 * sensorToMeters(rearRight.getSelectedSensorVelocity()));
    }

    public void resetOdometry(){
        m_odometry.resetPosition(gyroscope.getRotation2d(), 0, 0, getPose());
    }

    public double getGyroHeadingRadians(){
        return Units.degreesToRadians(gyroscope.getYaw());
    }
    
    public void zeroGyro(){
        gyroscope.setYaw(0);
    }

    @Override
    public void periodic(){
        
        m_odometry.update(gyroscope.getRotation2d(), sensorToMeters(rearLeft.getSelectedSensorPosition()), -1 * sensorToMeters(rearRight.getSelectedSensorPosition()));
    }

}
