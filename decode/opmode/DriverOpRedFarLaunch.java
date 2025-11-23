package org.firstinspires.ftc.teamcode.decode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_FAR_TARGET;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_START_FAR_LAUNCH;

@TeleOp(name="RED FAR DriverOp", group="DriverOp")
//@Disabled
public class DriverOpRedFarLaunch extends DriverOp {
    private final DecodeConstants.FieldPosition startPosition = RED_START_FAR_LAUNCH;
    private final DecodeConstants.FieldPosition targetPosition = RED_FAR_TARGET;


    @Override
    public void runOpMode() {
        super.initDecodeOp(hardwareMap, telemetry, DecodeConstants.TeamAllianceColor.RED_ALLIANCE, this.startPosition
                ,this.targetPosition, false);
        super.runOpMode();
    }
}