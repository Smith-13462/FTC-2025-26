package org.firstinspires.ftc.teamcode.decode.common;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static java.util.List.of;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.*;

//==============================BACK WALL====================================
//A6(BLUE TARGET)----------------------------------------------------------F6(RED TARGET)
//A5-----------------------------------------------------------------------F5
//A4-----------------------------------------------------------------------F4
//A3-----------------------------------------------------------------------F3
//A2-----------------------------------------------------------------------F2
//A1-----------------------------------------------------------------------F1
//==============================AUDIENCE WALL================================

public class DecodeConfig {
    private static final double BOT_LENGTH_IN_INCHES = 18, BOT_WIDTH_IN_INCHES = 18;
    public static final Pose RED_TARGET_POSE =
            new Pose(132, 6, Math.toRadians(45));
    public static final Pose BLUE_TARGET_POSE =
            new Pose( 132, 138, Math.toRadians(45));
    public static final Pose RED_PICKUP_GPP_SPIKE_POSE =
            new Pose(36, 41, 90);
    public static final Pose RED_PICKUP_PGP_SPIKE_POSE =
            new Pose(60, 41, 90);
    public static final Pose RED_PICKUP_PPG_SPIKE_POSE =
            new Pose(84, 41,  90);
    public static final Pose RED_PICKUP_LOADING_ZONE_POSE =
            new Pose(24, 11, 0);
    public static final Pose BLUE_PICKUP_GPP_SPIKE_POSE =
            new Pose(132, 138,-1);
    public static final Pose BLUE_PICKUP_PGP_SPIKE_POSE =
            new Pose(132, 138, -1);
    public static final Pose BLUE_PICKUP_PPG_SPIKE_POSE =
            new Pose(132, 138, -1);
    public static final Pose BLUE_PICKUP_LOADING_ZONE_POSE =
            new Pose( 132, 138, -1);
    public static final Pose RED_START_FAR_LAUNCH_POSE =
            new Pose( BOT_LENGTH_IN_INCHES / 2, 48 + (BOT_WIDTH_IN_INCHES / 2), 0);
    public static final Pose RED_START_DEPOT_F5_POSE =
            new Pose( 120 - (BOT_WIDTH_IN_INCHES / 2), 24- (BOT_LENGTH_IN_INCHES / 2), 90);
    public static final Pose BLUE_START_FAR_LAUNCH_POSE =
            new Pose( BOT_LENGTH_IN_INCHES / 2, 96 - (BOT_WIDTH_IN_INCHES / 2), 0);
    public static final Pose BLUE_START_DEPOT_A5_POSE =
            new Pose( 120 - (BOT_WIDTH_IN_INCHES / 2), 120 + (BOT_LENGTH_IN_INCHES / 2), -90);
    public static Pose RED_LAUNCH_D1_POSE =
            new Pose( 15, 57, -1);
    public static final Pose RED_LAUNCH_D4_POSE =
            new Pose( 87, 57, -1);
    public static final Pose RED_LAUNCH_D5_POSE =
            new Pose( 111, 57, -1);
    public static final Pose RED_LAUNCH_D6_POSE =
            new Pose( 135, 57, -1);
    public static final Pose RED_LAUNCH_E5_POSE =
            new Pose( 111, 33, -1);
    public static final Pose RED_LAUNCH_E6_POSE =
            new Pose( 135, 33, -1);
    public static final Pose BLUE_LAUNCH_C1_POSE =
            new Pose( 132, 138, -1);
    public static final Pose BLUE_LAUNCH_C4_POSE =
            new Pose( 132, 138, -1);
    public static final Pose BLUE_LAUNCH_C5_POSE =
            new Pose( 132, 138, -1);
    public static final Pose BLUE_LAUNCH_C6_POSE =
            new Pose( 132, 138, -1);
    public static final Pose BLUE_LAUNCH_B5_POSE =
            new Pose( 132, 138, -1);
    public static final Pose BLUE_LAUNCH_B6_POSE =
            new Pose( 132, 138, -1);
    private static HashMap<DecodeConstants.FieldPosition, Pose> fieldPositionToPoseMap = new HashMap<DecodeConstants.FieldPosition, Pose>();
    private static HashMap<String, ArrayList<Pose>> viaPositionsMap = new HashMap<String, ArrayList<Pose>>();
    private static String COLON = ":";

    DecodeConfig(){
        initBotPositionsMap();
        setViaPoints();
        setLaunchPositionsHeadings();
    }

    public void initBotPositionsMap() {
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

        viaPositionsMap.put(RED_LAUNCH_D1.toString() + COLON + RED_PICKUP_LOADING_ZONE.toString(),
                new ArrayList<>(List.of (
                        new Pose(14.5, 36, 179),
                        new Pose(14.5, 24, 179)
                )));

        /*
        viaPositionsMap.put(RED_LAUNCH_D1.toString() + COLON + RED_PICKUP_GPP_SPIKE_POSE.toString(),
                new ArrayList<>(List.of (
                        new Pose(14.5, 36, 179),
                        new Pose(14.5, 24, 179)
                )));

            */
    }

    public static ArrayList<Pose> getCurvePoints(DecodeConstants.FieldPosition fromPos, DecodeConstants.FieldPosition toPos){
        ArrayList<Pose> curvePoints = new ArrayList<>();
        curvePoints.set(0, fieldPositionToPoseMap.get(fromPos));
        ArrayList<Pose> intermediatePoints = viaPositionsMap.get(fromPos.toString() + COLON + toPos.toString());
        if(intermediatePoints != null) {
            curvePoints.addAll(intermediatePoints);
        }
        curvePoints.add(fieldPositionToPoseMap.get(toPos));

        return curvePoints;
    }

    public static void setOverrideViaPoints(String path, ArrayList<Pose> overrideControlPoints) {
        viaPositionsMap.put(path, overrideControlPoints);
    }

    private static void setLaunchPositionsHeadings(){
        if(RED_LAUNCH_D1_POSE.getHeading() == -1){
            setLaunchHeading(RED_TARGET_POSE, RED_LAUNCH_D1_POSE);
        }

        if(RED_LAUNCH_D4_POSE.getHeading() == -1){
            setLaunchHeading(RED_TARGET_POSE, RED_LAUNCH_D4_POSE);
        }

        if(RED_LAUNCH_D5_POSE.getHeading() == -1){
            setLaunchHeading(RED_TARGET_POSE, RED_LAUNCH_D5_POSE);
        }

        if(RED_LAUNCH_D6_POSE.getHeading() == -1){
            setLaunchHeading(RED_TARGET_POSE, RED_LAUNCH_D6_POSE);
        }

        if(RED_LAUNCH_E5_POSE.getHeading() == -1){
            setLaunchHeading(RED_TARGET_POSE, RED_LAUNCH_E5_POSE);
        }

        if(RED_LAUNCH_E6_POSE.getHeading() == -1){
            setLaunchHeading(RED_TARGET_POSE, RED_LAUNCH_E6_POSE);
        }

        if(BLUE_LAUNCH_C1_POSE.getHeading() == -1){
            setLaunchHeading(BLUE_TARGET_POSE, BLUE_LAUNCH_C1_POSE);
        }

        if(BLUE_LAUNCH_C4_POSE.getHeading() == -1){
            setLaunchHeading(BLUE_TARGET_POSE, BLUE_LAUNCH_C4_POSE);
        }

        if(BLUE_LAUNCH_C5_POSE.getHeading() == -1){
            setLaunchHeading(BLUE_TARGET_POSE, BLUE_LAUNCH_C5_POSE);
        }

        if(BLUE_LAUNCH_C6_POSE.getHeading() == -1){
            setLaunchHeading(BLUE_TARGET_POSE, BLUE_LAUNCH_C6_POSE);
        }

        if(BLUE_LAUNCH_B5_POSE.getHeading() == -1){
            setLaunchHeading(BLUE_TARGET_POSE, BLUE_LAUNCH_B5_POSE);
        }

        if(BLUE_LAUNCH_B6_POSE.getHeading() == -1){
            setLaunchHeading(BLUE_TARGET_POSE, BLUE_LAUNCH_B5_POSE);
        }
    }

    private static void setLaunchHeading(Pose targetPose, Pose launchPoint) {

        double launchAngle = 0;

        launchAngle = DecodeUtil.getLaunchHeading(targetPose, launchPoint);

        launchAngle = DecodeUtil.normalizeFieldAngle(launchAngle);

        launchPoint.setHeading(Math.toRadians(launchAngle));
    }
}