package org.firstinspires.ftc.teamcode.decode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.bot.DecodeBot;
import org.firstinspires.ftc.teamcode.decode.bot.DecodeMotion;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;
import org.firstinspires.ftc.teamcode.decode.common.DecodeUtil;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.DRIVER_OP_MODE;

@TeleOp(name="Basic:ZZZDoNOTUseCommonDriverOp", group="DriverOp")
//@Disabled
public abstract class DriverOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private static ElapsedTime timeSinceAutoPosRequest = new ElapsedTime();
    private ElapsedTime timeSinceLastAngleAdjustment = new ElapsedTime();
    private ElapsedTime timeSinceLastArtifactPush= new ElapsedTime();
    private ElapsedTime timeSinceLastRotate = new ElapsedTime();
    private ElapsedTime timeSinceLastTopVelocityAdjustment = new ElapsedTime();
    private ElapsedTime timeSinceLastBottomVelocityAdjustment = new ElapsedTime();
    public DecodeBot owlsRobot;
    double powerMultiplier = 0.7;
    private Pose startPose;
    private static boolean autoPositionState = false;
    double topVelocityAdjustment =0.5, bottomVelocityAdjustment =0.5, angleRequested=55;
    private DecodeConstants.OP_MODE opMode = DRIVER_OP_MODE;
    DecodeConstants.TeamAllianceColor teamAllianceColor = null;
    DecodeConfig decodeConfig = null;
    @Override
    public void runOpMode() {
        waitForStart();
        runtime.reset();
        this.autoPositionState = false;
        timeSinceAutoPosRequest.reset();
        timeSinceLastAngleAdjustment.reset();
        timeSinceLastArtifactPush.reset();
        timeSinceLastTopVelocityAdjustment.reset();
        timeSinceLastBottomVelocityAdjustment.reset();
        telemetry.addLine("Driver Op in progress..");
        telemetry.update();

        while (opModeIsActive())
        {
            //////////////////////Driver 1 (gamepad 1) request//////////////////
            //movement related
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            //double strafe1 = gamepad1.left_stick_x;
            double strafe1 = gamepad1.left_trigger;
            double strafe2 = gamepad1.right_trigger;
            if(DecodeConfig.REVERSE_DRIVE_BOOLEAN){
                drive = drive * -1;
                strafe1 = strafe1 * -1;
                strafe2 = strafe2 * -1;
            }
            if (gamepad1.a ){
                powerMultiplier = powerMultiplier + 0.05;
                if (powerMultiplier > 0.8) powerMultiplier = 0.8;
            } else if (gamepad1.b) {
                powerMultiplier = powerMultiplier - 0.05;
                if (powerMultiplier < 0.4) powerMultiplier = 0.4;
            }

            if(gamepad1.dpad_up || gamepad2.dpad_up) {
                if(!isDelayNeeded()) {
                    this.autoPositionState = true;
                    timeSinceAutoPosRequest.reset();
                    owlsRobot.getMotion().turnToLaunch(teamAllianceColor);
                }
            } else if(gamepad1.dpad_down || gamepad2.dpad_down) {
                owlsRobot.getMotion().turnOffPositionHold(teamAllianceColor);
                this.autoPositionState = false;
            }

            // for testing only - begin
            if(gamepad2.x) {
                if(timeSinceLastTopVelocityAdjustment.time() > 1) {
                    topVelocityAdjustment += 0.01;
                    if (topVelocityAdjustment > 1) {
                        topVelocityAdjustment = 1;
                    }
                    owlsRobot.getLauncher().adjustTopLaunchVelocity(topVelocityAdjustment);
                    timeSinceLastTopVelocityAdjustment.reset();
                }
            }  else if(gamepad2.y) {
                if(timeSinceLastTopVelocityAdjustment.time() > 1) {
                    topVelocityAdjustment -= 0.01;
                    if (topVelocityAdjustment < 0) {
                        topVelocityAdjustment = 0;
                    }
                    owlsRobot.getLauncher().adjustTopLaunchVelocity(topVelocityAdjustment);
                    timeSinceLastTopVelocityAdjustment.reset();
                }
            } else if (gamepad2.a) {
                if(timeSinceLastBottomVelocityAdjustment.time() > 1) {
                    bottomVelocityAdjustment += 0.01;
                    if (bottomVelocityAdjustment > 1) {
                        bottomVelocityAdjustment = 1;
                    }
                    owlsRobot.getLauncher().adjustBottomLaunchVelocity(bottomVelocityAdjustment);
                    timeSinceLastBottomVelocityAdjustment.reset();
                }
            } else if(gamepad2.b) {
                if(timeSinceLastBottomVelocityAdjustment.time() > 1) {
                    bottomVelocityAdjustment -= 0.01;
                    if (bottomVelocityAdjustment < 0) {
                        bottomVelocityAdjustment = 0;
                    }
                    owlsRobot.getLauncher().adjustBottomLaunchVelocity(bottomVelocityAdjustment);
                    timeSinceLastBottomVelocityAdjustment.reset();
                }
            }
            // for testing only - end

            if(gamepad2.left_bumper){
                if(timeSinceLastRotate.time() > 1) {
                    owlsRobot.getContainer().rotateContainer(false);
                    timeSinceLastRotate.reset();
                }
            } else if(gamepad2.right_bumper){
                if(timeSinceLastRotate.time() > 1) {
                    owlsRobot.getContainer().rotateContainer(false);
                    timeSinceLastRotate.reset();
                }
            }

            if(gamepad2.right_trigger > 0.1) {
                if(timeSinceLastArtifactPush.time() > 1) {
                    owlsRobot.getContainer().pushArtifactForLaunch(0);
                    timeSinceLastArtifactPush.reset();
                }
            }

           // telemetry.addLine(owlsRobot.getLauncher().getLauncherDisplayInfo());
           // telemetry.addLine(owlsRobot.getContainer().getContainerDisplayInfo());
            telemetry.addLine(owlsRobot.getMotion().getBotPositionDisplayInfo(teamAllianceColor, true));
            //telemetry.addLine(owlsRobot.getVision().getVisionDisplayInfo(teamAllianceColor));
            telemetry.update();

            //driver 1 movement, reset
            if(!isDelayNeeded()) {
                owlsRobot.manualSteer(drive, turn, strafe1, strafe2, powerMultiplier);
              //  owlsRobot.getMotion().getUpdatedBotPose();
            }
        }
    }
    public void initDecodeOp(HardwareMap hardwareMap, Telemetry telemetry, DecodeConstants.TeamAllianceColor teamAllianceColor, DecodeConstants.FieldPosition pStartPosition){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.teamAllianceColor = teamAllianceColor;

        this.autoPositionState = false;
        decodeConfig = new DecodeConfig();

        telemetry.addLine("Initializing Driver Op for " + teamAllianceColor.toString());

        if((DecodeMotion.getLastKnownAutoBotPose() != null) &&
                (DecodeMotion.getTimeSinceLastknownAutoBotPose().time() < 60)){
            this.startPose = DecodeMotion.getLastKnownAutoBotPose();
            telemetry.addLine("Start pose set from auto opmode last known " +
                    String.format("%6.1f ", DecodeMotion.getTimeSinceLastknownAutoBotPose().time()) + " seconds ago!");
            telemetry.addLine("Start pose set to " + DecodeMotion.getLastKnownAutoBotPose());
        } else {
            if(pStartPosition != null) {
                this.startPose = DecodeConfig.getFieldPositionPose(pStartPosition);

                if (pStartPosition.toString().toUpperCase().contains("RED_LAUNCH") ||
                        pStartPosition.toString().toUpperCase().contains("BLUE_LAUNCH")){
                    double heading = 0;
                    heading = DecodeUtil.getLaunchHeading(teamAllianceColor, this.startPose);
                    this.startPose = new Pose(this.startPose.getX(), this.startPose.getY(), heading);
                }
                telemetry.addLine("Start pose set to field position " + pStartPosition.toString());
            } else {
                this.startPose = null;
                telemetry.addLine("Start pose set to 0,0");
            }
        }

        owlsRobot = new DecodeBot(opMode, hardwareMap, telemetry,this.startPose, decodeConfig);
        telemetry.addLine("Ready to start Driver Op....");
        telemetry.update();
    }

    private boolean isDelayNeeded() {
        boolean delayNeeded = false;
        if(this.autoPositionState && (timeSinceAutoPosRequest.time() < 3.5)) {
            delayNeeded = true;
        }

        return delayNeeded;
    }
}