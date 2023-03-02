package frc.robot.commands.Autonomous.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPosition extends CommandBase {
    Arm arm;

    public ArmPosition(Arm arm, int pos){
        this.arm = arm;
    }

}
