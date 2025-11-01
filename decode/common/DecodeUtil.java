package org.firstinspires.ftc.teamcode.decode.common;

import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConfig.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.*;

public class DecodeUtil {
    public static double getLaunchHeading(DecodeConstants.TeamAllianceColor teamAllianceColor, Pose launchPoint) {
        double launchAngle = 0;

        Pose targetPose = (RED_ALLIANCE.equals(teamAllianceColor)) ? DecodeConfig.RED_TARGET_POSE : BLUE_TARGET_POSE;

        launchAngle = Math.atan((targetPose.getY() - launchPoint.getY()) /
                (targetPose.getX() - launchPoint.getX()));

        if(REVERSE_DRIVE_BOOLEAN) {
            launchAngle = Math.toRadians(normalizeFieldAngle(normalizeFieldAngle(Math.toDegrees(launchAngle)) + 180));
        } else {
            launchAngle = Math.toRadians(normalizeFieldAngle(Math.toDegrees(launchAngle)));
        }

        return  launchAngle;
    }

    public static double getLaunchHeadingAdjustment(DecodeConstants.TeamAllianceColor teamAllianceColor, Pose currentBotPose, double left, double forward) {
        double adjustAngle = 0;

        Pose targetPose = (RED_ALLIANCE.equals(teamAllianceColor)) ? DecodeConfig.RED_TARGET_POSE : BLUE_TARGET_POSE;

        adjustAngle = Math.atan((targetPose.getY() - currentBotPose.getY() - left)
                / (targetPose.getX()- currentBotPose.getX() - forward));

        if(DecodeConfig.REVERSE_DRIVE_BOOLEAN) {
            adjustAngle = normalizeFieldAngle(normalizeFieldAngle(Math.toDegrees(adjustAngle)) + 180);
        } else {
            adjustAngle = normalizeFieldAngle(Math.toDegrees(adjustAngle));
        }

        adjustAngle = Math.toRadians(normalizeFieldAngle(adjustAngle - normalizeFieldAngle(Math.toDegrees(currentBotPose.getHeading()))));

        return  adjustAngle;
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