package frc.robot.commands.Drivetrain;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DistanceToVisionTarget extends CommandBase {
    Drivetrain drivetrain;
    PIDController controller;
    PhotonCamera camera;
    double kp, kd, ki;
    double speed;

    public DistanceToVisionTarget(Drivetrain drivetrain, PhotonCamera camera){
        this.drivetrain = drivetrain;
        this.camera = camera;

        addRequirements(drivetrain);
        
        kp = .14;
        kd= 0;
        ki = 0;
    }

    @Override
    public void initialize(){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);

        controller = new PIDController(0, 0, 0);
        controller.setP(kp);
        controller.setD(kd);
        controller.setI(ki);

    }
    @Override
    public void execute(){
        if(RobotContainer.hasTargets(RobotContainer.result))
            speed = controller.calculate(RobotContainer.distanceToVisionTargetMeters(), 3);
        if(speed < 0){
            speed = Math.max(speed, -0.5);
        }
        else {
            speed = Math.min(speed, .5);
        }
        drivetrain.arcadeDrive(speed, 0);
    }
    @Override 
    public void end(boolean interrupted){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
