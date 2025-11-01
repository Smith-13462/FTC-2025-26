package org.firstinspires.ftc.teamcode.decode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;

import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_D1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_PICKUP_GPP_SPIKE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_START_FAR_LAUNCH;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.LaunchZone.*;

@Autonomous(name="RED FAR LAUNCH PICKUP GPP", group="AutoRedFarLaunchZone")
//@Disabled
public class AutoOpRedFarLaunchPickupGPP extends AutoOp {
    private final DecodeConstants.TeamAllianceColor teamAllianceColor = RED_ALLIANCE;
    private final DecodeConstants.LaunchZone launchZone = FAR_LAUNCH_ZONE;
    private final DecodeConstants.FieldPosition startPosition = RED_START_FAR_LAUNCH;
    private final DecodeConstants.FieldPosition launchPosition = RED_LAUNCH_D1;
    private final DecodeConstants.FieldPosition firstPickUpPosition = RED_PICKUP_GPP_SPIKE;
    private final DecodeConstants.FieldPosition secondPickUpPosition = null;
    private Pose overrideStartPose = null;
    private double minWaitBeforeFirstLaunch = 0, minWaitBeforeSecondLaunch = 60, minWaitBeforeThirdLaunch = 60;
    private double minWaitBeforeFirstPickup = 0, minWaitBeforeSecondPickup = 60;
    private HashMap<String, ArrayList<Pose>> overrideViaPointsMap = null;

    @Override
    public void runOpMode() {
        ElapsedTime autoOpElapsedTime = new ElapsedTime();
        setOverrideViaPoints();
        initAutoOp(hardwareMap ,this.teamAllianceColor ,launchZone ,startPosition
                ,overrideStartPose ,overrideViaPointsMap,
                firstPickUpPosition, secondPickUpPosition, launchPosition);
        waitForStart();
        autoOpElapsedTime.reset();
        launchAndPickupArtifacts(minWaitBeforeFirstLaunch - autoOpElapsedTime.time(),
                minWaitBeforeSecondLaunch - autoOpElapsedTime.time(),
                minWaitBeforeThirdLaunch - autoOpElapsedTime.time(),
                minWaitBeforeFirstPickup - autoOpElapsedTime.time(),
                minWaitBeforeSecondPickup - autoOpElapsedTime.time());
    }

    public void setOverrideViaPoints() {
    }
}