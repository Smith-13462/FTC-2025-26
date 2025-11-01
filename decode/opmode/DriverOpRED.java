package org.firstinspires.ftc.teamcode.decode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_D1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_START_FAR_LAUNCH;

@TeleOp(name="RED DriverOp", group="DriverOp")
//@Disabled
public class DriverOpRED extends DriverOp {
//    private final DecodeConstants.FieldPosition startPosition = RED_LAUNCH_D1;
    private final DecodeConstants.FieldPosition startPosition = RED_START_FAR_LAUNCH;

    @Override
    public void runOpMode() {
        super.initDecodeOp(hardwareMap, telemetry, DecodeConstants.TeamAllianceColor.RED_ALLIANCE, this.startPosition);
        super.runOpMode();
    }
}