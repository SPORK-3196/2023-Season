package frc.robot.commands.Autonomous.Cone;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Positions.ArmStationPosition;
import frc.robot.commands.Claw.OpenClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lift;

public class PickupConeStation extends SequentialCommandGroup {

    public PickupConeStation(Arm arm, Lift lift, Claw claw){
        super(
            new ArmStationPosition(arm, lift),
            new OpenClaw(claw)
        );
    }
}
