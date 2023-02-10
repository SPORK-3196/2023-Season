package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

    public WPI_TalonFX rearLeft = new WPI_TalonFX(DrivetrainConstants.rearLeftPort);
    public WPI_TalonFX frontLeft = new WPI_TalonFX(DrivetrainConstants.frontLeftPort);
    public WPI_TalonFX rearRight = new WPI_TalonFX(DrivetrainConstants.rearRightPort);
    public WPI_TalonFX frontRight = new WPI_TalonFX(DrivetrainConstants.frontRightPort);

    private WPI_PigeonIMU gyroscope = new WPI_PigeonIMU(DrivetrainConstants.gyroPort);

    private MotorControllerGroup leftGroup = new MotorControllerGroup(rearLeft, frontLeft);
    private MotorControllerGroup rightGroup = new MotorControllerGroup(rearRight, frontRight);

    public DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    
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
}
