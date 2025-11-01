package org.firstinspires.ftc.teamcode.decode.opmode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.bot.DecodeAction;
import org.firstinspires.ftc.teamcode.decode.bot.DecodeBot;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.bot.DecodeAction;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;
import org.firstinspires.ftc.teamcode.decode.common.DecodeUtil;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Motif.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.*;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous(name="ZZZZ Net Common - DO NOT USE", group="AutoLaunchZone")
//@TeleOp(name="TestLaunchZone", group="Linear Opmode")
//@Disabled
public abstract class AutoOp extends LinearOpMode {
    public DecodeConstants.TeamAllianceColor teamAllianceColor;
    public DecodeBot owlsRobot;
    private Pose startPose = null;
    private DecodeConstants.Motif detectedMotif = null;
    private DecodeAction decodeAction = null;
    private DecodeConstants.FieldPosition launchPosition;
    private DecodeConstants.FieldPosition firstPickUpPosition;
    private DecodeConstants.FieldPosition secondPickUpPosition;
    private String displayInitialInfo;

    public abstract void runOpMode();

    @SuppressLint("SuspiciousIndentation")
    public void initAutoOp(HardwareMap hardwareMap, DecodeConstants.TeamAllianceColor teamAllianceColor, DecodeConstants.LaunchZone launchZone,
                           FieldPosition startPosition, Pose overridestartPose, HashMap<String, ArrayList<Pose>> overrideViaPointsMap,
                           FieldPosition firstPickUpPosition, FieldPosition secondPickUpPosition,
                           FieldPosition launchPosition) {

        this.teamAllianceColor = teamAllianceColor;
        DecodeConfig decodeConfig = new DecodeConfig();
        if (overridestartPose != null) {
            this.startPose = overridestartPose;
        } else if (startPosition != null) {
            this.startPose = decodeConfig.getFieldPositionPose(startPosition);
        }
        this.launchPosition = launchPosition;
        this.firstPickUpPosition = firstPickUpPosition;
        this.secondPickUpPosition = secondPickUpPosition;

        if (overrideViaPointsMap != null) {
            overrideViaPointsMap.forEach((overrideViaPointsPath, overrideViaPoints) ->
                    decodeConfig.setOverrideViaPoints(overrideViaPointsPath, overrideViaPoints));
        }

        owlsRobot = new DecodeBot(AUTO_OP_MODE, hardwareMap, telemetry, this.startPose, decodeConfig);
        decodeAction = new DecodeAction(owlsRobot, telemetry);

        displayInitialInfo ="";
        displayInitialInfo += "\n" + "Initializing " + teamAllianceColor.toString() + " " + launchZone.toString() + "...";
        displayInitialInfo += "\n" + "Pickup Position(s): " +
                ((firstPickUpPosition == null) ? " " : firstPickUpPosition.toString()) +
                ((secondPickUpPosition == null) ? " " : "," + secondPickUpPosition.toString());

        //adjustStartPoseHeadingUsingTag(3);

        String displayMotif;
        while (this.opModeInInit()) {
          displayMotif = detectMotifUntilStart();
            telemetry.addLine(displayInitialInfo +
                    "\n" +  displayMotif +
                    "\n" +  "Waiting to start... ");
            telemetry.update();
        }
    }

    private void adjustStartPoseHeadingUsingTag(double maxDelta) {
        Pose tagPose = null;
        ElapsedTime holdclock = new ElapsedTime();
        holdclock.reset();
        tagPose = owlsRobot.getVision().getPedroPoseUsingTargetTag(teamAllianceColor);
        while(holdclock.time() < 30) {
            tagPose = owlsRobot.getVision().getPedroPoseUsingTargetTag(teamAllianceColor);
            if (tagPose == null) {
                telemetry.addLine("Tag pose heading to adjust not available");
                telemetry.update();
            } else {
                telemetry.addLine("Tag pose heading=" + DecodeUtil.normalizeFieldAngle(Math.toDegrees(tagPose.getHeading())));
                telemetry.update();
            }
        }

        /*
        if (tagPose == null) return;

        if (Math.abs(tagPose.getHeading() - this.startPose.getHeading()) < maxDelta) {
            Pose adjustedPose = new Pose(this.startPose.getX(), this.startPose.getY(),
                    tagPose.getHeading());

            this.startPose = adjustedPose;
            owlsRobot.getMotion().resetPose(this.startPose);
            telemetry.addLine("Bot pose heading adjusted to " + Math.toDegrees(this.startPose.getHeading()));
        } else {
            telemetry.addLine("Bot pose heading not adjusted ");
        }
        */
    }

    public String detectMotifUntilStart() {
        String displayMessage = "";
        try {
            this.detectedMotif = owlsRobot.getVision().getDetectedMotif();
            if (detectedMotif.equals(UNKNOWN_MOTIF)) {
                this.detectedMotif = PGP_MOTIF;
                displayMessage = "Motif not visible. Defaulted to PGP";
            } else {
                telemetry.addLine();
                displayMessage = "Detected Motif: " + detectedMotif;
            }
        } catch (Exception ignored) {
            detectedMotif = PGP_MOTIF;
            displayMessage = "Problem detecting Motif. Defaulted to PGP";
        }

        return displayMessage;
    }

    public void launchAndPickupArtifacts(double waitBeforeFirstLaunch, double waitBeforeSecondLaunch, double waitBeforeThirdLaunch,
                                         double waitBeforeFirstPickup, double waitBeforeSecondPickup) {
        ElapsedTime holdClock = new ElapsedTime();
        holdClock.reset();

        telemetry.addLine("Traveling to launch position " + launchPosition.toString());
        telemetry.update();

        owlsRobot.getMotion().travelToLaunchOrPickup(TravelPathShape.CURVE, launchPosition, false, null);
        owlsRobot.getMotion().turnToLaunch(teamAllianceColor);

        if (waitBeforeFirstLaunch > 30) {
            //do nothing
        } else {
            if (waitBeforeFirstLaunch > 0) {
                while (holdClock.time() < waitBeforeFirstLaunch) {
                    telemetry.addLine("Waiting to launch....");
                    telemetry.update();
                }
            }
            telemetry.addLine("Launching artifacts....");
            telemetry.update();
            owlsRobot.getAction().autoLaunch(detectedMotif);
        }

        if (firstPickUpPosition != null) {
            if (waitBeforeFirstPickup > 30) {
                //do nothing
            } else {
                if (waitBeforeFirstPickup > 0) {
                    while (holdClock.time() < waitBeforeFirstPickup) {
                        telemetry.addLine("Waiting to travel for pickup from " + firstPickUpPosition.toString());
                        telemetry.update();
                    }
                }
                telemetry.addLine("Traveling to pickup from " + firstPickUpPosition.toString());
                telemetry.update();
                owlsRobot.getMotion().travelToLaunchOrPickup(TravelPathShape.CURVE, firstPickUpPosition, false, null);

                telemetry.addLine("Picking up artifacts....");
                telemetry.update();
                //owlsRobot.getIntake().autoPickup(firstPickUpPosition);
                owlsRobot.getMotion().moveStraightToPickup(teamAllianceColor, firstPickUpPosition, 1);

                ElapsedTime timer = new ElapsedTime();
                timer.reset();
                while(timer.time() < 10){
                    telemetry.addLine(owlsRobot.getMotion().getBotPositionDisplayInfo(teamAllianceColor, true));
                    telemetry.update();
                }

/*
                telemetry.addLine("Traveling to launch position " + launchPosition.toString());
                telemetry.update();
                owlsRobot.getMotion().travelToLaunchOrPickup(TravelPathShape.CURVE, launchPosition, false, null);

                if (waitBeforeSecondLaunch > 30) {
                    //do nothing
                } else {
                    if (waitBeforeSecondLaunch > 0) {
                        while (holdClock.time() < waitBeforeSecondLaunch) {
                            telemetry.addLine("Waiting to launch....");
                            telemetry.update();
                        }
                    }
                    owlsRobot.getAction().autoLaunch(detectedMotif);
                    telemetry.addLine("Launching artifacts....");
                    telemetry.update();
                }
            }

            if (secondPickUpPosition != null) {
                if (waitBeforeSecondPickup > 30) {
                    //do nothing
                } else {
                    if (waitBeforeSecondPickup > 0) {
                        while (holdClock.time() < waitBeforeSecondPickup) {
                            telemetry.addLine("Waiting to travel for pickup from " + firstPickUpPosition.toString());
                            telemetry.update();
                        }
                    }
                    telemetry.addLine("Traveling to pickup from " + secondPickUpPosition.toString());
                    telemetry.update();
                    owlsRobot.getMotion().travelToLaunchOrPickup(TravelPathShape.CURVE, secondPickUpPosition, false, null);
                    owlsRobot.getIntake().autoPickup(secondPickUpPosition);

                    telemetry.addLine("Traveling to launch position " + launchPosition.toString());
                    telemetry.update();
                    telemetry.addLine("Picking up artifacts....");
                    telemetry.update();
                    owlsRobot.getMotion().travelToLaunchOrPickup(TravelPathShape.CURVE, launchPosition, false, null);
                    if (waitBeforeThirdLaunch > 30) {
                        //do nothing
                    } else {
                        if (waitBeforeThirdLaunch > 0) {
                            while (holdClock.time() < waitBeforeThirdLaunch) {
                                telemetry.addLine("Waiting to launch....");
                                telemetry.update();
                            }
                        }
                        telemetry.addLine("Launching artifacts....");
                        telemetry.update();
                        owlsRobot.getAction().autoLaunch(detectedMotif);
                    }
                }

                 */
            }

            //for teleop
            owlsRobot.getMotion().getLastKnownAutoBotPose();
        }
    }
}