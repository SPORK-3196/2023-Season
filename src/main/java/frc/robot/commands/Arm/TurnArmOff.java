package frc.robot.commands.Arm;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TurnArmOff extends CommandBase{
    Arm arm;
    
    public TurnArmOff(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void initialize() {
        arm.turnElbowOff();
        arm.turnShoulderOff();
        arm.elbowMotor.setIdleMode(IdleMode.kBrake);
        arm.elbowMotor.setIdleMode(IdleMode.kBrake);
    }
    @Override
    public void execute() {

    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true

    public void end() {
        //System.out.println("Finished using manual PID arm control...");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

    public void interrupted() {
    }
}