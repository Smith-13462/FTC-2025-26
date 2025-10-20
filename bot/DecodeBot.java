package org.firstinspires.ftc.teamcode.decode.bot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.*;
import com.pedropathing.geometry.Pose;

public class DecodeBot extends DOwlsBotBase {
    private DecodeVision owlsVision = null;
    private static DecodeMotion owlsMotion = null;
    private static ArtifactLauncher owlsLauncher = null;
    private static ArtifactIntake owlsIntake = null;
    private static ArtifactContainer owlsContainer = null;

    public DecodeBot(DecodeConstants.OP_MODE owlsOpMode, HardwareMap hardwareMap, Telemetry telemetry, Pose startPose) {
        super();
        if (owlsOpMode == DRIVER_OP_MODE) {
            Initialize_TeleOp_Robot(hardwareMap, telemetry);
            InitializeBotForDecode(hardwareMap, true, false, owlsOpMode, startPose);
        } else if (owlsOpMode == AUTO_OP_MODE) {
            Initialize_Autonomous_Robot(hardwareMap, telemetry);
            InitializeBotForDecode(hardwareMap, true, true, owlsOpMode, startPose);
        }
    }
    public void InitializeBotForDecode(HardwareMap hardwareMap, boolean automatedDrive, boolean recalibratePinpoint,
                                       DecodeConstants.OP_MODE opMode, Pose startPose) {
        owlsVision = new DecodeVision(hardwareMap);
        owlsMotion = new DecodeMotion(hardwareMap, this, automatedDrive, recalibratePinpoint,
                startPose, telemetry);
        owlsLauncher = new ArtifactLauncher(hardwareMap, opMode, telemetry);
        owlsIntake = new ArtifactIntake(hardwareMap, opMode, telemetry);
        owlsContainer = new ArtifactContainer(hardwareMap, opMode, telemetry);
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

    public ArtifactIntake getIntake() {
        return owlsIntake;
    }

    public ArtifactContainer getContainer() {
        return owlsContainer;
    }

}