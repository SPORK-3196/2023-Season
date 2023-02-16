package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ArmStationPosition extends CommandBase{
     Elevator elevator;
     Arm arm;
     public ArmStationPosition(Arm arm, Elevator elevator){
        this.arm = arm;
        this.elevator = elevator;
     }
}
