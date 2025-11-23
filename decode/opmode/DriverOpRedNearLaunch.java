package org.firstinspires.ftc.teamcode.decode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_NEAR_TARGET;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_START_DEPOT_F5;
@TeleOp(name="RED NEAR DriverOp", group="DriverOp")
//@Disabled
public class DriverOpRedNearLaunch extends DriverOp {
    private final DecodeConstants.FieldPosition startPosition = RED_START_DEPOT_F5;
    private final DecodeConstants.FieldPosition targetPosition = RED_NEAR_TARGET;


    @Override
    public void runOpMode() {
        super.initDecodeOp(hardwareMap, telemetry, DecodeConstants.TeamAllianceColor.RED_ALLIANCE, this.startPosition
                ,this.targetPosition, false);
        super.runOpMode();
    }
}