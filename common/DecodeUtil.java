package org.firstinspires.ftc.teamcode.decode.common;

import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConfig.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.*;

public class DecodeUtil {
    public static double getLaunchHeading(DecodeConstants.TeamAllianceColor teamAllianceColor, Pose botPose) {
        double launchAngle = 0;

        Pose targetPose = (RED_ALLIANCE.equals(teamAllianceColor)) ? RED_TARGET_POSE : BLUE_TARGET_POSE;

        launchAngle = getLaunchHeading(targetPose, botPose);

        return  launchAngle;
    }

    public static double getLaunchHeading(Pose targetPose, Pose launchpoint) {

        double launchAngle = 0;

        launchAngle = Math.toDegrees(Math.atan((targetPose.getY() - launchpoint.getY())
                / (targetPose.getX()- launchpoint.getX())));

        launchAngle = normalizeFieldAngle(launchAngle);

        return launchAngle;
    }

    public static double getLaunchHeadingAdjustment(DecodeConstants.TeamAllianceColor teamAllianceColor, Pose currentBotPose, double left, double forward) {
        double adjustAngle = 0;

        Pose targetPose = (RED_ALLIANCE.equals(teamAllianceColor)) ? RED_TARGET_POSE : BLUE_TARGET_POSE;

        adjustAngle = Math.toDegrees(Math.atan((targetPose.getY() - currentBotPose.getY() - left)
                / (targetPose.getX()- currentBotPose.getX() - forward)))
                - Math.toDegrees(currentBotPose.getHeading());

        return  DecodeUtil.normalizeFieldAngle(adjustAngle);
    }

    public static double normalizeFieldAngle(double angle){
        double normalizedAngle = angle;

        if(normalizedAngle > 180) {
            normalizedAngle = (360 - normalizedAngle) * -1;
        } else if(normalizedAngle < -180) {
            normalizedAngle = 360 + normalizedAngle;
        }

        return normalizedAngle;
    }
}