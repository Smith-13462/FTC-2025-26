package org.firstinspires.ftc.teamcode.decode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;

import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_END_FAR_LAUNCH;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_FAR_TARGET;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_C1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_PICKUP_GPP_SPIKE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_START_FAR_LAUNCH;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.LaunchZone.*;

@Autonomous(name="BLUE FAR LAUNCH PICKUP GPP", group="AutoBlueFarLaunchZone")
//@Disabled
public class AutoOpBlueFarLaunchPickupGPP extends AutoOp {
    private final DecodeConstants.TeamAllianceColor teamAllianceColor = BLUE_ALLIANCE;
    private final DecodeConstants.LaunchZone launchZone = FAR_LAUNCH_ZONE;
    private final DecodeConstants.FieldPosition startPosition = BLUE_START_FAR_LAUNCH;
    private final DecodeConstants.FieldPosition launchPosition = BLUE_LAUNCH_C1;
    private final DecodeConstants.FieldPosition firstPickUpPosition = BLUE_PICKUP_GPP_SPIKE;
    private final DecodeConstants.FieldPosition secondPickUpPosition = null;
    private final DecodeConstants.FieldPosition endPosition = BLUE_END_FAR_LAUNCH;
    private final DecodeConstants.FieldPosition targetPosition = BLUE_FAR_TARGET;
    private Pose overrideStartPose = null;
    private double minWaitBeforeFirstLaunch = 0, minWaitBeforeSecondLaunch = 0, minWaitBeforeThirdLaunch = 60;
    private double minWaitBeforeFirstPickup = 0, minWaitBeforeSecondPickup = 60;
    private double maxWaitTimeForSpeedAdjustment = 2.25;
    private HashMap<String, ArrayList<Pose>> overrideViaPointsMap = null;

    @Override
    public void runOpMode() {
        ElapsedTime autoOpElapsedTime = new ElapsedTime();
        setOverrideViaPoints();
        initAutoOp(hardwareMap ,this.teamAllianceColor ,launchZone ,startPosition
                ,overrideStartPose ,overrideViaPointsMap,
                firstPickUpPosition, secondPickUpPosition, launchPosition, endPosition
                ,targetPosition, maxWaitTimeForSpeedAdjustment, false);
        waitForStart();
        autoOpElapsedTime.reset();

        launchAndPickupArtifacts(minWaitBeforeFirstLaunch - autoOpElapsedTime.time(),
                minWaitBeforeSecondLaunch - autoOpElapsedTime.time(),
                minWaitBeforeThirdLaunch - autoOpElapsedTime.time(),
                minWaitBeforeFirstPickup - autoOpElapsedTime.time(),
                minWaitBeforeSecondPickup - autoOpElapsedTime.time(), 28.5 - autoOpElapsedTime.time());

        leaveLaunchArea();

        wrapupForHandOffToTeleOp();
    }

    public void setOverrideViaPoints() {
    }
}