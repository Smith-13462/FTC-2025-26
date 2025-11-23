package org.firstinspires.ftc.teamcode.decode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_NEAR_TARGET;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_START_DEPOT_A5;

@TeleOp(name="BLUE NEAR DriverOp", group="DriverOp")
//@Disabled
public class DriverOpBlueNearLaunch extends DriverOp {
    private final DecodeConstants.FieldPosition startPosition = BLUE_START_DEPOT_A5;
    private final DecodeConstants.FieldPosition targetPosition = BLUE_NEAR_TARGET;

    @Override
    public void runOpMode() {
        super.initDecodeOp(hardwareMap, telemetry, DecodeConstants.TeamAllianceColor.BLUE_ALLIANCE, this.startPosition
        ,this.targetPosition, true);
        super.runOpMode();
    }
}