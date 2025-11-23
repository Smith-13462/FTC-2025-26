package org.firstinspires.ftc.teamcode.decode.bot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.*;
import com.pedropathing.geometry.Pose;

public class DecodeBot extends DOwlsBotBase {
    private DecodeVision owlsVision = null;
    private static DecodeMotion owlsMotion = null;
    private static DecodeAction owlsAction = null;
    private static ArtifactLauncher owlsLauncher = null;
    private static ArtifactIntake owlsIntake = null;
    private static ArtifactContainer owlsContainer = null;
    private static DecodeConfig decodeConfig = null;

    public DecodeBot(DecodeConstants.OP_MODE owlsOpMode, HardwareMap hardwareMap, Telemetry telemetry, Pose startPose,
                     DecodeConfig decodeConfig, DecodeConstants.TeamAllianceColor teamAllianceColor
    , DecodeConstants.FieldPosition targetPosition, boolean nearLaunchArea) {
        super();
        if (owlsOpMode == DRIVER_OP_MODE) {
            Initialize_TeleOp_Robot(hardwareMap, telemetry);
            InitializeBotForDecode(hardwareMap, true, false, owlsOpMode, startPose, decodeConfig,
                    teamAllianceColor, targetPosition, nearLaunchArea);
        } else if (owlsOpMode == AUTO_OP_MODE) {
            Initialize_Autonomous_Robot(hardwareMap, telemetry);
            InitializeBotForDecode(hardwareMap, true, true, owlsOpMode, startPose, decodeConfig,
                    teamAllianceColor, targetPosition, nearLaunchArea);
        }
    }
    public void InitializeBotForDecode(HardwareMap hardwareMap, boolean automatedDrive, boolean recalibratePinpoint,
                                       DecodeConstants.OP_MODE opMode, Pose startPose, DecodeConfig decodeConfig,
                                       DecodeConstants.TeamAllianceColor teamAllianceColor
    , DecodeConstants.FieldPosition targetPosition, boolean nearLaunchArea) {
        owlsVision = new DecodeVision(hardwareMap, telemetry, teamAllianceColor);
        owlsMotion = new DecodeMotion(hardwareMap, this, automatedDrive, recalibratePinpoint,
                startPose, telemetry, opMode, teamAllianceColor, targetPosition);
        owlsLauncher = new ArtifactLauncher(hardwareMap, opMode, telemetry, teamAllianceColor, targetPosition, nearLaunchArea);
        owlsIntake = new ArtifactIntake(hardwareMap, opMode, telemetry);
        owlsContainer = new ArtifactContainer(hardwareMap, opMode, telemetry);
        owlsAction = new DecodeAction(this, telemetry, teamAllianceColor);
        this.decodeConfig = decodeConfig;
    }
    public DecodeVision getVision() {
        return owlsVision;
    }

    public DecodeMotion getMotion() {
        return owlsMotion;
    }

    public ArtifactLauncher getLauncher() {
        return owlsLauncher;
    }

    public DecodeAction getAction() {
        return owlsAction;
    }

    public ArtifactIntake getIntake() {
        return owlsIntake;
    }

    public ArtifactContainer getContainer() {
        return owlsContainer;
    }

    public DecodeConfig getConfig() {
        return decodeConfig;
    }
}