package org.firstinspires.ftc.teamcode.decode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;

import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_FAR_TARGET;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_END_FAR_LAUNCH;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_END_NEAR_LAUNCH;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_D5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_NEAR_TARGET;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_PICKUP_PPG_SPIKE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_START_DEPOT_F5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.LaunchZone.*;

@Autonomous(name="RED NEAR LAUNCH PICKUP PPG", group="AutoRedNearLaunchZone")
//@Disabled
public class AutoOpRedNearLaunchPickupPPG extends AutoOp {
    private final DecodeConstants.TeamAllianceColor teamAllianceColor = RED_ALLIANCE;
    private final DecodeConstants.LaunchZone launchZone = NEAR_LAUNCH_ZONE;
    private final DecodeConstants.FieldPosition startPosition = RED_START_DEPOT_F5;
    private final DecodeConstants.FieldPosition launchPosition = RED_LAUNCH_D5;
    private final DecodeConstants.FieldPosition firstPickUpPosition = RED_PICKUP_PPG_SPIKE;
    private final DecodeConstants.FieldPosition secondPickUpPosition = null;
    private final DecodeConstants.FieldPosition endPosition = RED_END_NEAR_LAUNCH;
    private final DecodeConstants.FieldPosition targetPosition = RED_NEAR_TARGET;


    private Pose overrideStartPose = null;
    private double minWaitBeforeFirstLaunch = 0, minWaitBeforeSecondLaunch = 0, minWaitBeforeThirdLaunch = 60;
    private double minWaitBeforeFirstPickup = 0, minWaitBeforeSecondPickup = 60;
    private double maxWaitTimeForSpeedAdjustment = 1;
    private HashMap<String, ArrayList<Pose>> overrideViaPointsMap = null;

    @Override
    public void runOpMode() {
        ElapsedTime autoOpElapsedTime = new ElapsedTime();
        setOverrideViaPoints();
        initAutoOp(hardwareMap ,this.teamAllianceColor ,launchZone ,startPosition
                ,overrideStartPose ,overrideViaPointsMap,
                firstPickUpPosition, secondPickUpPosition, launchPosition, endPosition
                ,targetPosition, maxWaitTimeForSpeedAdjustment, true);
        waitForStart();
        autoOpElapsedTime.reset();
        launchAndPickupArtifacts(minWaitBeforeFirstLaunch - autoOpElapsedTime.time(),
                minWaitBeforeSecondLaunch - autoOpElapsedTime.time(),
                minWaitBeforeThirdLaunch - autoOpElapsedTime.time(),
                minWaitBeforeFirstPickup - autoOpElapsedTime.time(),
                minWaitBeforeSecondPickup - autoOpElapsedTime.time(), 29 - autoOpElapsedTime.time());

        leaveLaunchArea();

        wrapupForHandOffToTeleOp();
    }

    public void setOverrideViaPoints() {
    }
}