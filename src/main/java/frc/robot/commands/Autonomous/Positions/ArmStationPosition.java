package frc.robot.commands.Autonomous.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ArmStationPosition extends CommandBase{
     Elevator lift;
     Arm arm;
     public ArmStationPosition(Arm arm, Elevator lift){
        this.arm = arm;
        this.lift = lift;
     }
}
