package frc.robot.commands.Autonomous.Cube;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TrackToCube extends CommandBase{
    Drivetrain drivetrain;
    PIDController controller;
    double kp, kd, ki;
    double speedFiltered, rotationFiltered, boost, gyroAngle;

    public TrackToCube(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        
        kp = .0075;
        kd = .005;
        ki = 0;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);

        controller = new PIDController(kp, ki, kd);
        controller.setTolerance(.5);
        
        RobotContainer.raspiCam.setPipelineIndex(0);
        RobotContainer.raspiCam.setDriverMode(false);

    }

    @Override
    public void execute(){
        speedFiltered = RobotContainer.LJSY_Primary * 0.8;
        if(OI.Vision.piCamHasTargets)
            gyroAngle = RobotContainer.getCamYaw(RobotContainer.piBResult);
        double speed = -controller.calculate(gyroAngle);

        if(speed > .25){
            speed = .25;
        }
        else if(speed < -.25){
            speed = -.25;
        }

        drivetrain.arcadeDriveAI(speedFiltered, speed);
    }
    
    @Override
    public void end(boolean isFinished){
        RobotContainer.raspiCam.setDriverMode(false);

    }

    @Override
    public boolean isFinished(){
        return controller.atSetpoint();
    }
}