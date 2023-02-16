package frc.robot.commands.Autonomous.Cone;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.ArmStationPosition;
import frc.robot.commands.Claw.OpenClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class PickupConeStation extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;

    public PickupConeStation(Arm arm, Elevator elevator, Claw claw){
        super(
            new ArmStationPosition(arm, elevator),
            new OpenClaw(claw)
        );
    }
}
