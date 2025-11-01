package org.firstinspires.ftc.teamcode.decode.common;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static java.util.List.of;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.BLUE_ALLIANCE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.RED_ALLIANCE;

//==============================BACK WALL====================================
//A6(BLUE TARGET)----------------------------------------------------------F6(RED TARGET)
//A5-----------------------------------------------------------------------F5
//A4-----------------------------------------------------------------------F4
//A3-----------------------------------------------------------------------F3
//A2-----------------------------------------------------------------------F2
//A1-----------------------------------------------------------------------F1
//==============================AUDIENCE WALL================================

public class DecodeConfig {
    public static double BOT_LENGTH_IN_INCHES = 17, BOT_WIDTH_IN_INCHES = 17, REVERSE_DRIVE = 1;
    public static boolean REVERSE_DRIVE_BOOLEAN = true;
    //RED target pose
    public static Pose RED_TARGET_POSE =
            new Pose(132, 6, Math.toRadians(45));
    //RED start poses
    public static Pose RED_START_FAR_LAUNCH_POSE =
            new Pose( 0 + (BOT_LENGTH_IN_INCHES / 2), 47.25 + BOT_WIDTH_IN_INCHES, Math.toRadians(0 + (180 * REVERSE_DRIVE)));
    public static Pose RED_START_DEPOT_F5_POSE =
            new Pose( 121 - (BOT_WIDTH_IN_INCHES / 2), 6.25 + BOT_LENGTH_IN_INCHES, Math.toRadians(90 - (180 * REVERSE_DRIVE)));
    //RED launch poses - do not set heading;  computed during init
    public static Pose RED_LAUNCH_D1_POSE =
            new Pose(21 , 64.25 , Math.toRadians(0)); //56.25
    public static Pose RED_LAUNCH_D4_POSE =
            new Pose( 87, 64.25, Math.toRadians(0));
    public static Pose RED_LAUNCH_D5_POSE =
            new Pose( 111, 64.25, Math.toRadians(0));
    public static Pose RED_LAUNCH_D6_POSE =
            new Pose( 135, 64.25, Math.toRadians(0));
    public static Pose RED_LAUNCH_E5_POSE =
            new Pose( 111, 40, Math.toRadians(0));
    public static Pose RED_LAUNCH_E6_POSE =
            new Pose( 135, 40, Math.toRadians(0));
    //RED pickup poses
    public static Pose RED_PICKUP_LOADING_ZONE_POSE =
            new Pose(16, 23, Math.toRadians(60 - (180 * REVERSE_DRIVE)));
    public static Pose RED_PICKUP_GPP_SPIKE_POSE =
            new Pose(36, 54, Math.toRadians(90 - (180 * REVERSE_DRIVE)));
    public static Pose RED_PICKUP_PGP_SPIKE_POSE =
            new Pose(60, 54, Math.toRadians(90 - (180 * REVERSE_DRIVE)));
    public static Pose RED_PICKUP_PPG_SPIKE_POSE =
            new Pose(84, 54,  Math.toRadians(90 - (180 * REVERSE_DRIVE)));
    //TBD: Blue poses to be adjusted
    //BLUE target pose
    public static Pose BLUE_TARGET_POSE =
            new Pose( 132, 138, Math.toRadians(45));
    //BLUE start poses
    public static Pose BLUE_START_FAR_LAUNCH_POSE =
            new Pose( BOT_LENGTH_IN_INCHES / 2, 96 - (BOT_WIDTH_IN_INCHES / 2), 0);
    public static Pose BLUE_START_DEPOT_A5_POSE =
            new Pose( 120 - (BOT_WIDTH_IN_INCHES / 2), 120 + (BOT_LENGTH_IN_INCHES / 2), -90);
    //BLUE launch poses - do not set heading;  computed during init
    public static Pose BLUE_LAUNCH_C1_POSE =
            new Pose( 132, 138, Math.toRadians(0));
    public static Pose BLUE_LAUNCH_C4_POSE =
            new Pose( 132, 138, Math.toRadians(0));
    public static Pose BLUE_LAUNCH_C5_POSE =
            new Pose( 132, 138, Math.toRadians(0));
    public static Pose BLUE_LAUNCH_C6_POSE =
            new Pose( 132, 138, Math.toRadians(0));
    public static Pose BLUE_LAUNCH_B5_POSE =
            new Pose( 132, 138, Math.toRadians(0));
    public static Pose BLUE_LAUNCH_B6_POSE =
            new Pose( 132, 138, Math.toRadians(0));
    //Blue pickup poses
    public static Pose BLUE_PICKUP_LOADING_ZONE_POSE =
            new Pose( 132, 138, Math.toRadians(-90 + (180 * REVERSE_DRIVE)));
    public static Pose BLUE_PICKUP_GPP_SPIKE_POSE =
            new Pose(132, 138, Math.toRadians(-90 + (180 * REVERSE_DRIVE)));
    public static Pose BLUE_PICKUP_PGP_SPIKE_POSE =
            new Pose(132, 138, Math.toRadians(-90 + (180 * REVERSE_DRIVE)));
    public static Pose BLUE_PICKUP_PPG_SPIKE_POSE =
            new Pose(132, 138, Math.toRadians(-90 + (180 * REVERSE_DRIVE)));
    private static HashMap<DecodeConstants.FieldPosition, Pose> fieldPositionToPoseMap = new HashMap<DecodeConstants.FieldPosition, Pose>();
    private static HashMap<String, ArrayList<Pose>> viaPositionsMap = new HashMap<String, ArrayList<Pose>>();
    private static final String COLON = ":";

    public DecodeConfig(){
        setLaunchPositionsHeadings();
        initBotPositionsMap();
        setViaPoints();
    }

    public static void initBotPositionsMap() {
        fieldPositionToPoseMap = new HashMap<DecodeConstants.FieldPosition, Pose>();
        fieldPositionToPoseMap.put(RED_PICKUP_GPP_SPIKE, RED_PICKUP_GPP_SPIKE_POSE);
        fieldPositionToPoseMap.put(RED_PICKUP_PGP_SPIKE, RED_PICKUP_PGP_SPIKE_POSE);
        fieldPositionToPoseMap.put(RED_PICKUP_PPG_SPIKE, RED_PICKUP_PPG_SPIKE_POSE);
        fieldPositionToPoseMap.put(RED_PICKUP_LOADING_ZONE, RED_PICKUP_LOADING_ZONE_POSE); //For RED alliance during auto
        fieldPositionToPoseMap.put(BLUE_PICKUP_GPP_SPIKE, BLUE_PICKUP_GPP_SPIKE_POSE);
        fieldPositionToPoseMap.put(BLUE_PICKUP_PGP_SPIKE, BLUE_PICKUP_PGP_SPIKE_POSE);
        fieldPositionToPoseMap.put(BLUE_PICKUP_PPG_SPIKE, BLUE_PICKUP_PPG_SPIKE_POSE);
        fieldPositionToPoseMap.put(BLUE_PICKUP_LOADING_ZONE, BLUE_PICKUP_LOADING_ZONE_POSE); //For BLUE alliance during auto
        fieldPositionToPoseMap.put(RED_START_FAR_LAUNCH, RED_START_FAR_LAUNCH_POSE);
        fieldPositionToPoseMap.put(RED_START_DEPOT_F5, RED_START_DEPOT_F5_POSE);
        fieldPositionToPoseMap.put(BLUE_START_FAR_LAUNCH, BLUE_START_FAR_LAUNCH_POSE);
        fieldPositionToPoseMap.put(BLUE_START_DEPOT_A5, BLUE_START_DEPOT_A5_POSE);
        fieldPositionToPoseMap.put(RED_LAUNCH_D1, RED_LAUNCH_D1_POSE);
        fieldPositionToPoseMap.put(RED_LAUNCH_D4, RED_LAUNCH_D4_POSE);
        fieldPositionToPoseMap.put(RED_LAUNCH_D5, RED_LAUNCH_D5_POSE);
        fieldPositionToPoseMap.put(RED_LAUNCH_D6, RED_LAUNCH_D6_POSE);
        fieldPositionToPoseMap.put(RED_LAUNCH_E5, RED_LAUNCH_E5_POSE);
        fieldPositionToPoseMap.put(RED_LAUNCH_E6, RED_LAUNCH_E6_POSE);
        fieldPositionToPoseMap.put(BLUE_LAUNCH_C1, BLUE_LAUNCH_C1_POSE);
        fieldPositionToPoseMap.put(BLUE_LAUNCH_C4, BLUE_LAUNCH_C4_POSE);
        fieldPositionToPoseMap.put(BLUE_LAUNCH_C5, BLUE_LAUNCH_C5_POSE);
        fieldPositionToPoseMap.put(BLUE_LAUNCH_C6, BLUE_LAUNCH_C6_POSE);
        fieldPositionToPoseMap.put(BLUE_LAUNCH_B5, BLUE_LAUNCH_B5_POSE);
        fieldPositionToPoseMap.put(BLUE_LAUNCH_B6, BLUE_LAUNCH_B6_POSE);
    }

    public static Pose getFieldPositionPose(DecodeConstants.FieldPosition fieldPosition) {
        return fieldPositionToPoseMap.get(fieldPosition);
    }

    public static HashMap<DecodeConstants.FieldPosition, Pose> getFieldPositionMap() {
        return fieldPositionToPoseMap;
    }

    private static void setViaPoints(){
        /*
        viaPositionsMap.put(RED_LAUNCH_D1.toString() + COLON + RED_PICKUP_LOADING_ZONE.toString(),
                new ArrayList<>(List.of (
                        new Pose(24 + (BOT_LENGTH_IN_INCHES / 2), 48 + (BOT_WIDTH_IN_INCHES / 2), 0)
                )));

        viaPositionsMap.put(RED_LAUNCH_D1.toString() + COLON + RED_PICKUP_GPP_SPIKE_POSE.toString(),
                new ArrayList<>(List.of (
                        new Pose(14.5, 36, 179),
                        new Pose(14.5, 24, 179)
                )));

            */
    }

    public static ArrayList<Pose> getCurvePoints(DecodeConstants.FieldPosition fromPos, DecodeConstants.FieldPosition toPos){
        ArrayList<Pose> curvePoints = new ArrayList<>();
        if(fieldPositionToPoseMap.get(fromPos) != null) {
            curvePoints.set(0, fieldPositionToPoseMap.get(fromPos));
        }
        ArrayList<Pose> intermediatePoints = viaPositionsMap.get(fromPos.toString() + COLON + toPos.toString());
        if(intermediatePoints != null) {
            curvePoints.addAll(intermediatePoints);
        }
        if(fieldPositionToPoseMap.get(toPos) != null) {
            curvePoints.add(fieldPositionToPoseMap.get(toPos));
        } else {
            curvePoints.clear();
        }

        return curvePoints;
    }

    public static void setOverrideViaPoints(String path, ArrayList<Pose> overrideControlPoints) {
        viaPositionsMap.put(path, overrideControlPoints);
    }

    private static void setLaunchPositionsHeadings(){
        if(RED_LAUNCH_D1_POSE.getHeading() == 0) {
            RED_LAUNCH_D1_POSE = RED_LAUNCH_D1_POSE.setHeading(DecodeUtil.getLaunchHeading(RED_ALLIANCE, RED_LAUNCH_D1_POSE));
        }

        if(RED_LAUNCH_D4_POSE.getHeading() == 0){
            RED_LAUNCH_D4_POSE = RED_LAUNCH_D4_POSE.setHeading(DecodeUtil.getLaunchHeading(RED_ALLIANCE, RED_LAUNCH_D4_POSE));
        }

        if(RED_LAUNCH_D5_POSE.getHeading() == 0){
            RED_LAUNCH_D5_POSE = RED_LAUNCH_D5_POSE.setHeading(DecodeUtil.getLaunchHeading(RED_ALLIANCE, RED_LAUNCH_D5_POSE));
        }

        if(RED_LAUNCH_D6_POSE.getHeading() == 0){
            RED_LAUNCH_D6_POSE = RED_LAUNCH_D6_POSE.setHeading(DecodeUtil.getLaunchHeading(RED_ALLIANCE, RED_LAUNCH_D6_POSE));
        }

        if(RED_LAUNCH_E5_POSE.getHeading() == 0){
            RED_LAUNCH_E5_POSE = RED_LAUNCH_E5_POSE.setHeading(DecodeUtil.getLaunchHeading(RED_ALLIANCE, RED_LAUNCH_E5_POSE));
        }

        if(RED_LAUNCH_E6_POSE.getHeading() == 0){
            RED_LAUNCH_E6_POSE = RED_LAUNCH_E6_POSE.setHeading(DecodeUtil.getLaunchHeading(RED_ALLIANCE, RED_LAUNCH_E6_POSE));
        }

        if(BLUE_LAUNCH_C1_POSE.getHeading() == 0){
            BLUE_LAUNCH_C1_POSE = BLUE_LAUNCH_C1_POSE.setHeading(DecodeUtil.getLaunchHeading(BLUE_ALLIANCE, BLUE_LAUNCH_C1_POSE));
        }

        if(BLUE_LAUNCH_C4_POSE.getHeading() == 0){
            BLUE_LAUNCH_C4_POSE = BLUE_LAUNCH_C4_POSE.setHeading(DecodeUtil.getLaunchHeading(BLUE_ALLIANCE, BLUE_LAUNCH_C4_POSE));
        }

        if(BLUE_LAUNCH_C5_POSE.getHeading() == 0){
            BLUE_LAUNCH_C5_POSE = BLUE_LAUNCH_C5_POSE.setHeading(DecodeUtil.getLaunchHeading(BLUE_ALLIANCE, BLUE_LAUNCH_C5_POSE));
        }

        if(BLUE_LAUNCH_C6_POSE.getHeading() == 0){
            BLUE_LAUNCH_C6_POSE = BLUE_LAUNCH_C6_POSE.setHeading(DecodeUtil.getLaunchHeading(BLUE_ALLIANCE, BLUE_LAUNCH_C6_POSE));
        }

        if(BLUE_LAUNCH_B5_POSE.getHeading() == 0){
            BLUE_LAUNCH_B5_POSE = BLUE_LAUNCH_B5_POSE.setHeading(DecodeUtil.getLaunchHeading(BLUE_ALLIANCE, BLUE_LAUNCH_B5_POSE));
        }

        if(BLUE_LAUNCH_B6_POSE.getHeading() == 0){
            BLUE_LAUNCH_B6_POSE = BLUE_LAUNCH_B6_POSE.setHeading(DecodeUtil.getLaunchHeading(BLUE_ALLIANCE, BLUE_LAUNCH_B6_POSE));
        }
    }
}