package org.firstinspires.ftc.teamcode.decode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

@TeleOp(name="RED DriverOp", group="DriverOp")
//@Disabled
public class DriverOpRED extends DriverOp {
    private Pose startPose = new Pose(9, 57+0,  0);

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Red Alliance Driver Op... ");
        telemetry.update();
        super.initDecodeOp(hardwareMap, telemetry, DecodeConstants.TeamAllianceColor.RED_ALLIANCE, this.startPose);
        telemetry.addLine("Ready to start Driver Op....");
        telemetry.update();
        super.runOpMode();
    }
}