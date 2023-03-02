package frc.robot.commands.Autonomous.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ElevatorPosition extends CommandBase{
     Elevator elevator;
     Arm arm;
     public ElevatorPosition(Arm arm, Elevator elevator){
        this.arm = arm;
        this.elevator = elevator;
     }
}
