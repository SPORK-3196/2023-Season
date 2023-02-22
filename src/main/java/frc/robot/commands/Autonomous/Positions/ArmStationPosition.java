package frc.robot.commands.Autonomous.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;

public class ArmStationPosition extends CommandBase{
     Lift lift;
     Arm arm;
     public ArmStationPosition(Arm arm, Lift lift){
        this.arm = arm;
        this.lift = lift;
     }
}
