package org.firstinspires.ftc.teamcode.decode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_FAR_TARGET;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_START_FAR_LAUNCH;

@TeleOp(name="BLUE FAR DriverOp", group="DriverOp")
//@Disabled
public class DriverOpBlueFarLaunch extends DriverOp {
    private final DecodeConstants.FieldPosition startPosition = BLUE_START_FAR_LAUNCH;
    private final DecodeConstants.FieldPosition targetPosition = BLUE_FAR_TARGET;

    @Override
    public void runOpMode() {
        super.initDecodeOp(hardwareMap, telemetry, DecodeConstants.TeamAllianceColor.BLUE_ALLIANCE, this.startPosition
        ,this.targetPosition, false);
        super.runOpMode();
    }
}