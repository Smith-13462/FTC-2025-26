package org.firstinspires.ftc.teamcode.decode.common;

public class DecodeConstants {
    public enum TeamAllianceColor {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    public enum OP_MODE {
        DRIVER_OP_MODE,
        AUTO_OP_MODE,
        DRIVER_AUTO_OP_MODE
    }

    public enum ArtifactColor {
        PURPLE_ARTIFACT,
        GREEN_ARTIFACT,
        UNKNOWN_ARTIFACT,
        NO_ARTIFACT
    }

    public enum TravelPathShape {
        DIAGONAL,
        CURVE,
        L_X_Y,
        L_Y_X
    }

    public enum LaunchZone {
        NEAR_LAUNCH_ZONE,
        FAR_LAUNCH_ZONE
    }

    public enum CameraPosition {
        LEFT_CAMERA,
        RIGHT_CAMERA,
        FRONT_CAMERA,
        REAR_CAMERA
    }

    public enum LimeLightPipeline {
        APRILTAG_PIPELINE
    }

    public enum FieldTarget {
        RED_FAR_TARGET,
        RED_NEAR_TARGET,
        BLUE_FAR_TARGET,
        BLUE_NEAR_TARGET,
        UNKNOWN_TARGET
    }

    public enum Motif {
        PGP_MOTIF,
        GPP_MOTIF,
        PPG_MOTIF,
        UNKNOWN_MOTIF
    }

    public enum AprilTagPosition {
        MOTIF_PGP_TAG,
        MOTIF_GPP_TAG,
        BLUE_TARGET_TAG,
        RED_TARGET_TAG,
        MOTIF_PPG_TAG
    }

    public enum FieldPosition {
        RED_PICKUP_GPP_SPIKE,
        RED_PICKUP_PGP_SPIKE,
        RED_PICKUP_PPG_SPIKE,
        RED_PICKUP_LOADING_ZONE,  //For red alliance in autonomous
        BLUE_PICKUP_GPP_SPIKE,
        BLUE_PICKUP_PGP_SPIKE,
        BLUE_PICKUP_PPG_SPIKE,
        BLUE_PICKUP_LOADING_ZONE,  //For blue alliance in autonomous
        RED_START_FAR_LAUNCH,
        RED_END_NEAR_LAUNCH,
        RED_END_FAR_LAUNCH,
        RED_START_DEPOT_F5,
        BLUE_START_FAR_LAUNCH,
        BLUE_END_FAR_LAUNCH,
        BLUE_END_NEAR_LAUNCH,
        BLUE_START_DEPOT_A5,
        RED_LAUNCH_D1,
        BLUE_LAUNCH_D1,
        RED_LAUNCH_D4,
        RED_LAUNCH_C4,
        RED_LAUNCH_D5,
        RED_LAUNCH_C5,
        RED_LAUNCH_B5,
        RED_LAUNCH_D6,
        RED_LAUNCH_E5,
        RED_LAUNCH_E6,
        BLUE_LAUNCH_C1,
        RED_LAUNCH_C1,
        BLUE_LAUNCH_C4,
        BLUE_LAUNCH_D4,
        BLUE_LAUNCH_C5,
        BLUE_LAUNCH_D5,
        BLUE_LAUNCH_E5,
        BLUE_LAUNCH_C6,
        BLUE_LAUNCH_B5,
        BLUE_LAUNCH_B6,
        RED_NEAR_VERTEX,
        BLUE_NEAR_VERTEX,
        RED_FAR_TARGET,
        RED_NEAR_TARGET,
        BLUE_FAR_TARGET,
        BLUE_NEAR_TARGET
    }

    public enum Action {
        JUST_LAUNCH,
        AUTO_LAUNCH,
        TURN_TO_LAUNCH,
        AUTOOP_LAUNCH
    }

    public enum SpeedUnit {
        TICKS_PER_SECOND,
        ROTATIONS_PER_MINUTE
    }

}