package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotContainer;

public class TracktoTag extends CommandBase {
    Drivetrain drivetrain;
    PIDController controller;
    double rotPower;
    double yaw;
    double kp, kd, ki;

    public TracktoTag(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        kp = .014;
        kd = .0004;
        ki = .005;
    }

    @Override
    public void initialize(){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
        
        //controller = new PIDController(.014, .005, .0004);
        controller = new PIDController(0, 0, 0);
        controller.setP(kp);
        controller.setD(kd);
        controller.setI(ki);

        if(RobotContainer.aprilYaw > 1) 
            rotPower -= .02;
        
        else if(RobotContainer.aprilYaw < 1)
            rotPower += .02;
    }

    @Override
    public void execute(){
        if(RobotContainer.hasTargets(RobotContainer.result))
            rotPower = controller.calculate(RobotContainer.aprilYaw, 0);

        drivetrain.arcadeDriveAI(0, rotPower);
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
