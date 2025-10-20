package org.firstinspires.ftc.teamcode.decode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import java.util.ArrayList;
import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_START_FAR_LAUNCH;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.LaunchZone.*;

@Autonomous(name="RED FAR LAUNCH ZONE", group="AutoRedLaunchZone")
//@TeleOp(name="TestRedLaunchZone", group="Linear Opmode")
//@Disabled
public class AutoOpRedFarLaunchZone extends AutoOp {
    private final DecodeConstants.TeamAllianceColor teamAllianceColor = RED_ALLIANCE;
    private final DecodeConstants.LaunchZone launchZone = FAR_LAUNCH_ZONE;
    private final DecodeConstants.FieldPosition startPosition = RED_START_FAR_LAUNCH;
    private Pose overrideStartPose = null;
    private double minWaitBeforeFirstLaunch = 0, minWaitBeforeSecondLaunch = 0;
    private double minWaitBeforeFirstPickup = 0, minWaitBeforeSecondPickup = 0;
    private HashMap<String, ArrayList<Pose>> overrideViaPointsMap = null;

    @Override
    public void runOpMode() {
        ElapsedTime autoOpElapsedTime = new ElapsedTime();
        telemetry.addLine("Initializing " + teamAllianceColor.toString() + " " + launchZone.toString() + "...");
        telemetry.update();
        setOverrideViaPoints();
        initAutoOp(hardwareMap ,this.teamAllianceColor ,launchZone ,startPosition ,overrideStartPose ,overrideViaPointsMap);
        telemetry.addLine("Waiting to start " + teamAllianceColor.toString() + " " + launchZone.toString() + "...");
        telemetry.update();
        waitForStart();
        autoOpElapsedTime.reset();
        launchAndPickupArtifacts(minWaitBeforeFirstLaunch - autoOpElapsedTime.time(),
                minWaitBeforeSecondLaunch - autoOpElapsedTime.time());
    }

    public void setOverrideViaPoints() {
    }
}