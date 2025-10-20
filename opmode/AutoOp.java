package org.firstinspires.ftc.teamcode.decode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.bot.DecodeBot;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

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

    public abstract void runOpMode();
    public void initAutoOp(HardwareMap hardwareMap, DecodeConstants.TeamAllianceColor teamAllianceColor, DecodeConstants.LaunchZone launchZone,
                           FieldPosition startPosition, Pose overridestartPose , HashMap<String, ArrayList<Pose>> overrideViaPointsMap){

        this.teamAllianceColor = teamAllianceColor;
        if(overridestartPose != null) {
            this.startPose = overridestartPose;
        } else if(startPosition != null){
            this.startPose = DecodeConfig.getFieldPositionPose(startPosition);
        }

        if(overrideViaPointsMap != null){
            overrideViaPointsMap.forEach((overrideViaPointsPath, overrideViaPoints) ->
                DecodeConfig.setOverrideViaPoints(overrideViaPointsPath, overrideViaPoints));
        }

        owlsRobot = new DecodeBot(AUTO_OP_MODE, hardwareMap, telemetry, this.startPose);

        //syncBotStartPoseUsingTargetAprilTag();

        detectMotifUntilStart();
    }

    private void syncBotStartPoseUsingTargetAprilTag(){
        Pose2D botPose2D = null;
        botPose2D = owlsRobot.getVision().getLLBotPose();

        if(botPose2D != null){
           Pose botPose = new Pose(botPose2D.getX(DistanceUnit.INCH), botPose2D.getX(DistanceUnit.INCH),
                    botPose2D.getHeading(AngleUnit.RADIANS));
            this.startPose = botPose;
            owlsRobot.getMotion().resetPose(botPose);
            telemetry.addLine("Bot Pose initialized using tag to X=" + this.startPose.getX() +
                    " ,Y=" + this.startPose.getY() + " ,Angle=" + Math.toDegrees(botPose.getHeading()));
            telemetry.update();
        } else {
            telemetry.addLine("Tag not found during initialization");
            telemetry.update();
        }
    }

    public void detectMotifUntilStart(){

        while (this.opModeInInit()) {
            try {
                this.detectedMotif = owlsRobot.getVision().getDetectedMotif();
                if(detectedMotif.equals(UNKNOWN_MOTIF)) {
                    this.detectedMotif = PGP_MOTIF;
                    telemetry.addLine("Motif not visible. Defaulted to PGP");
                } else {
                    telemetry.addLine("Detected Motif: " + detectedMotif);
                }
                telemetry.update();
            } catch (Exception ignored) {
                detectedMotif = PGP_MOTIF;
                telemetry.addLine("Problem detecting Motif. Defaulted to PGP");
                telemetry.update();
            }
        }
    }

    public void launchAndPickupArtifacts(double waitBeforeFirstLaunch, double waitBeforeSecondLaunch) {
        ElapsedTime holdClock = new ElapsedTime();
        holdClock.reset();

        owlsRobot.getMotion().travelToLaunchOrPickup(TravelPathShape.CURVE, RED_START_FAR_LAUNCH, RED_LAUNCH_D1, false);

        if(waitBeforeFirstLaunch > 0) {
            while(holdClock.time() < waitBeforeFirstLaunch){}
        };

        owlsRobot.getLauncher().autoLaunch(detectedMotif);

//        owlsRobot.getMotion().travelToLaunchOrPickup(TravelPathShape.CURVE, RED_PICKUP_LOADING_ZONE, false);
//        owlsRobot.getMotion().travelToLaunchOrPickup(TravelPathShape.CURVE, RED_PICKUP_GPP_SPIKE, false);

//        owlsRobot.getIntake().autoPickup(RED_PICKUP_LOADING_ZONE);
//        owlsRobot.getIntake().autoPickup(RED_PICKUP_GPP_SPIKE);

//        owlsRobot.getMotion().travelToLaunchOrPickup(TravelPathShape.CURVE, RED_LAUNCH_D1, false);

        if(waitBeforeSecondLaunch > 0) {
            while(holdClock.time() < waitBeforeSecondLaunch){}
        };

        owlsRobot.getLauncher().autoLaunch(detectedMotif);
    }
}