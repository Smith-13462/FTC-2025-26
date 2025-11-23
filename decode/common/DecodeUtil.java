package org.firstinspires.ftc.teamcode.decode.common;

import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConfig.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_B5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_C1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_C4;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_C5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_D1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_D4;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_D5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_E5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_NEAR_VERTEX;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_B5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_C1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_C4;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_C5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_D1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_D4;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_D5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_E5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_NEAR_VERTEX;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.*;

public class DecodeUtil {
    public static double getLaunchHeading(DecodeConstants.TeamAllianceColor teamAllianceColor, Pose launchPoint,
                                          DecodeConstants.FieldPosition targetPosition) {
        double launchAngle = 0;

        double xDisplacement = 0, yDisplacement = 0;
        if (BLUE_ALLIANCE.equals(teamAllianceColor)) {
            xDisplacement *= -1;
        }

        Pose targetPose = DecodeConfig.getFieldPositionPose(targetPosition);

        launchAngle = Math.atan((targetPose.getY() - launchPoint.getY() - yDisplacement) /
                (targetPose.getX() - launchPoint.getX() - xDisplacement));

        if (REVERSE_DRIVE_BOOLEAN) {
            launchAngle = Math.toRadians(normalizeFieldAngle(normalizeFieldAngle(Math.toDegrees(launchAngle)) + 180));
        } else {
            launchAngle = Math.toRadians(normalizeFieldAngle(Math.toDegrees(launchAngle)));
        }

        return launchAngle;
    }

    public static double getLaunchHeadingAdjustment(DecodeConstants.TeamAllianceColor teamAllianceColor, Pose currentBotPose,
                                                    DecodeConstants.FieldPosition targetPosition, double left, double forward) {
        double adjustAngle = 0;

        Pose targetPose = DecodeConfig.getFieldPositionPose(targetPosition);

        double xDisplacement = 0, yDisplacement = 0;
        xDisplacement = 0;
        yDisplacement = 0;
        //        xDisplacement = 3 - (((launchPoint.getX() - 127) / 127) * 3);
//        yDisplacement = 3 - xDisplacement;

        if (BLUE_ALLIANCE.equals(teamAllianceColor)) {
            xDisplacement *= -1;
        }

        adjustAngle = Math.atan((targetPose.getY() - currentBotPose.getY() - yDisplacement - left)
                / (targetPose.getX() - currentBotPose.getX() - xDisplacement - forward));

        if (DecodeConfig.REVERSE_DRIVE_BOOLEAN) {
            adjustAngle = normalizeFieldAngle(normalizeFieldAngle(Math.toDegrees(adjustAngle)) + 180);
        } else {
            adjustAngle = normalizeFieldAngle(Math.toDegrees(adjustAngle));
        }

        adjustAngle = Math.toRadians(normalizeFieldAngle(adjustAngle - normalizeFieldAngle(Math.toDegrees(currentBotPose.getHeading()))));

        return adjustAngle;
    }

    public static double normalizeFieldAngle(double angle) {
        double normalizedAngle = angle;

        if (normalizedAngle > 180) {
            normalizedAngle = (360 - normalizedAngle) * -1;
        } else if (normalizedAngle < -180) {
            normalizedAngle = 360 + normalizedAngle;
        }

        return normalizedAngle;
    }

    public static double getDiagonalDistanceToTarget(DecodeConstants.TeamAllianceColor teamAllianceColor, Pose currentBotPose
            , DecodeConstants.FieldPosition targetPosition) {
        Pose targetPose = DecodeConfig.getFieldPositionPose(targetPosition);
        double diagonalDistance = 0;

        if (targetPose == null) return diagonalDistance;

        double xDisplacement = 0, yDisplacement = 0;
        xDisplacement = 3 - (((currentBotPose.getX() - 127) / 127) * 3);
        yDisplacement = 3 - xDisplacement;
        if (BLUE_ALLIANCE.equals(teamAllianceColor)) {
            xDisplacement *= -1;
        }

        diagonalDistance = Math.sqrt(Math.pow(targetPose.getX() - currentBotPose.getX() - xDisplacement, 2)
                + Math.pow(targetPose.getY() - currentBotPose.getY() - yDisplacement, 2));

        return diagonalDistance;
    }

    public static DecodeConstants.FieldPosition getLaunchPositionName(Pose currentPose, DecodeConstants.TeamAllianceColor teamAllianceColor) {

        if (currentPose == null) return null;

        //Far launch area
        if (currentPose.getX() <= 48) {
            if (RED_ALLIANCE.equals(teamAllianceColor)) {
                if ((currentPose.getY() >= 24) && (currentPose.getY() <= 66)) {
                    return RED_LAUNCH_D1;
                } else if ((currentPose.getY() > 66) && (currentPose.getY() <= 120)) {
                    return RED_LAUNCH_C1;
                }
            } else if (BLUE_ALLIANCE.equals(teamAllianceColor)) {
                    if ((currentPose.getY() >= 24) && (currentPose.getY() <= 72)) {
                        return BLUE_LAUNCH_D1;
                    } else if ((currentPose.getY() > 72) && (currentPose.getY() <= 120)) {
                        return BLUE_LAUNCH_C1;
                    }
                }
            }

        //Near launch area
        if(currentPose.getX() > 48){
            if (RED_ALLIANCE.equals(teamAllianceColor)) {
                if(currentPose.getX() < 72){
                    return RED_NEAR_VERTEX;
                } else if(currentPose.getX() < 80) {
                    if((currentPose.getY() <= 68)) {
                        return RED_LAUNCH_D4;
                    } else {
                        return RED_LAUNCH_C4;
                    }
                } else if(currentPose.getX() < 96) {

                    if((currentPose.getY() <= 46)) {
                        return RED_LAUNCH_E5;
                    } else if((currentPose.getY() > 46) && (currentPose.getY() <= 70)) {
                        //return RED_LAUNCH_D5;
                    } else if((currentPose.getY() > 70) && (currentPose.getY() <= 94)) {
                        //return RED_LAUNCH_C5;
                    } else if(currentPose.getY() > 94) {
                       // return RED_LAUNCH_B5;
                    }
                }
            } else if (BLUE_ALLIANCE.equals(teamAllianceColor)) {
                if(currentPose.getX() < 76) {
                    return BLUE_NEAR_VERTEX;
                } else if(currentPose.getX() < 88) {
                    if(currentPose.getY() <= 64) {
                        return BLUE_LAUNCH_D4;
                    } else {
                        return BLUE_LAUNCH_C4;
                    }
                } else if(currentPose.getX() < 110) {

                    if((currentPose.getY() <= 54)) {
                       // return BLUE_LAUNCH_E5;
                    } else if((currentPose.getY() > 54) && (currentPose.getY() <= 78)) {
                        //return BLUE_LAUNCH_D5;
                    } else if((currentPose.getY() > 78) && (currentPose.getY() <= 102)) {
                        //return BLUE_LAUNCH_C5;
                    } else if(currentPose.getY() > 102) {
                        return BLUE_LAUNCH_B5;
                    }

                }
            }
        }

            return null;
        }

}