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

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.GREEN_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.PURPLE_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.UNKNOWN_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.DRIVER_OP_MODE;

@TeleOp(name="Basic:ZZZDoNOTUseCommonDriverOp", group="DriverOp")
//@Disabled
public abstract class DriverOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //testing only - start
    private ElapsedTime timeSinceLastTopVelocityAdjustment = new ElapsedTime();
    private ElapsedTime timeSinceLastBottomVelocityAdjustment = new ElapsedTime();
    private ElapsedTime timeSinceLastPickupAlignment = new ElapsedTime();
    double topVelocityAdjustment =0.33, bottomVelocityAdjustment =0.33;
    // testing only - end
    public DecodeBot owlsRobot;
    double powerMultiplier = 0.5;
    double timeBetweenPickupAutoAlign = 0.5;
    private Pose startPose;
    private DecodeConstants.OP_MODE opMode = DRIVER_OP_MODE;
    DecodeConstants.TeamAllianceColor teamAllianceColor = null;
    DecodeConstants.FieldPosition targetPosition = null;
    DecodeConfig decodeConfig = null;
    @Override
    public void runOpMode() {
        waitForStart();
        runtime.reset();
        timeSinceLastTopVelocityAdjustment.reset();
        timeSinceLastBottomVelocityAdjustment.reset();
        timeSinceLastPickupAlignment.reset();
        telemetry.addLine("Driver Op in progress..");
        telemetry.update();

        while (opModeIsActive()) {
            //////////////////////Driver 1 (gamepad 1) request//////////////////
            //movement related
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            //double strafe1 = gamepad1.left_stick_x;
            double strafe1 = gamepad1.left_trigger;
            double strafe2 = gamepad1.right_trigger;
            if (DecodeConfig.REVERSE_DRIVE_BOOLEAN) {
                drive = drive * -1;
                strafe1 = strafe1 * -1;
                strafe2 = strafe2 * -1;
            }
            if (gamepad1.a) {
                powerMultiplier = powerMultiplier + 0.05;
                if (powerMultiplier > 0.8) powerMultiplier = 0.8;
            } else if (gamepad1.b) {
                powerMultiplier = powerMultiplier - 0.05;
                if (powerMultiplier < 0.4) powerMultiplier = 0.4;
            }

            // for testing only - begin
            if(gamepad1.x){
               owlsRobot.getLauncher().reverseTopMotorDirection(true);
            } else if(gamepad1.y) {
                owlsRobot.getLauncher().reverseTopMotorDirection(false);
            }

            if (gamepad2.x) {
                if(timeSinceLastTopVelocityAdjustment.time() > 0.25) {
                    topVelocityAdjustment += 0.01;
                    if (topVelocityAdjustment > 1) {
                        topVelocityAdjustment = 1;
                    }
                    owlsRobot.getLauncher().adjustTopLaunchVelocityForTest(topVelocityAdjustment);
                    timeSinceLastTopVelocityAdjustment.reset();
                }
            } else if (gamepad2.y) {
                if(timeSinceLastTopVelocityAdjustment.time() > 0.25) {
                    topVelocityAdjustment -= 0.01;
                    if (topVelocityAdjustment < 0) {
                        topVelocityAdjustment = 0;
                    }
                    owlsRobot.getLauncher().adjustTopLaunchVelocityForTest(topVelocityAdjustment);
                    timeSinceLastTopVelocityAdjustment.reset();
                }
            }

            if(gamepad2.a){
                if(timeSinceLastBottomVelocityAdjustment.time() > 0.25) {
                    bottomVelocityAdjustment += 0.01;
                    if (bottomVelocityAdjustment > 1) {
                        bottomVelocityAdjustment = 1;
                    }
                    owlsRobot.getLauncher().adjustBottomLaunchVelocityForTest(bottomVelocityAdjustment);
                    timeSinceLastBottomVelocityAdjustment.reset();
                }
            } else if(gamepad2.b){
                if(timeSinceLastBottomVelocityAdjustment.time() > 0.25) {
                    bottomVelocityAdjustment -= 0.01;
                    if (bottomVelocityAdjustment < 0) {
                        bottomVelocityAdjustment = 0;
                    }
                    owlsRobot.getLauncher().adjustBottomLaunchVelocityForTest(bottomVelocityAdjustment);
                    timeSinceLastBottomVelocityAdjustment.reset();
                }
            }
            // for testing only - end

            if(gamepad2.left_stick_button){
                owlsRobot.getAction().autoLaunchArtifact(UNKNOWN_ARTIFACT, 0, null,false,
                        false,false, 2.5, true);
                owlsRobot.getAction().rotateToPrepareForNextLaunch();
            } else if(gamepad2.right_stick_button){
                owlsRobot.getAction().autoLaunchAll(null, 15, false
                        , 2.5, true);
            } else if(gamepad2.left_trigger > 0.1) {
                owlsRobot.getAction().autoLaunchArtifact(GREEN_ARTIFACT, 0, null,false,
                        false,false, 2.5, true);
            } else if(gamepad2.right_trigger > 0.1) {
                owlsRobot.getAction().autoLaunchArtifact(PURPLE_ARTIFACT, 0, null,false,
                        false,false, 2.5, true);
            } else if(gamepad2.dpad_up){
                owlsRobot.getAction().turnToLaunch();
            } else if(gamepad2.dpad_down){
                owlsRobot.getAction().justLaunch();
            } else if(gamepad1.dpad_up) {
                owlsRobot.getAction().autoLaunchArtifact(UNKNOWN_ARTIFACT, 0, null, false,
                        false,true, 2.5, true);
            } else if(gamepad1.dpad_left){
                owlsRobot.getAction().autoLaunchArtifact(GREEN_ARTIFACT, 0, null,false,
                        false,true, 2.5,true);
            } else if(gamepad1.dpad_right){
                owlsRobot.getAction().autoLaunchArtifact(PURPLE_ARTIFACT, 0, null,false,
                        false,true, 2.5, true);
            }  else if(gamepad1.dpad_down){
                owlsRobot.getAction().autoLaunchAll(null, 15, false, 2.5
                , true);
            }

            if(gamepad2.dpad_left){
                owlsRobot.getContainer().rotateContainer(false, false);
            } else if(gamepad2.dpad_right){
                owlsRobot.getContainer().rotateContainer(true, false);
            }

            if(gamepad2.left_bumper) {
                owlsRobot.getIntake().turnOnIntake();
                owlsRobot.getLauncher().setDefaultPropulsionSpeed();
            } else if(gamepad2.right_bumper){
                owlsRobot.getIntake().turnOffIntake();
                owlsRobot.getLauncher().turnOffLaunchPower();
                owlsRobot.getLauncher().setDefaultPropulsionSpeed();
            }

            if(gamepad1.left_bumper) {
                owlsRobot.getIntake().turnOnIntake();
                owlsRobot.getLauncher().setLoadArtifactsSpeed();
            }else if(gamepad1.right_bumper) {
                owlsRobot.getIntake().turnOffIntake();
                owlsRobot.getLauncher().turnOffLaunchPower();
                owlsRobot.getLauncher().setDefaultPropulsionSpeed();
            }

            if(owlsRobot.getIntake().isIntakeActive()) {
                if(timeSinceLastPickupAlignment.time() >timeBetweenPickupAutoAlign) {
                    boolean rotateDone = false;
                    rotateDone = owlsRobot.getAction().alignEmptySlotToPickup(false);
                    if(rotateDone){
                        timeSinceLastPickupAlignment.reset();
                    }
                }
            }

            telemetry.addLine(owlsRobot.getLauncher().getLauncherDisplayInfo());
            telemetry.addLine(owlsRobot.getContainer().getContainerDisplayInfo());
            telemetry.addLine(owlsRobot.getMotion().getBotPositionDisplayInfo(teamAllianceColor, true));
            //telemetry.addLine(owlsRobot.getVision().getVisionDisplayInfo(teamAllianceColor));
            telemetry.update();

            //driver 1 movement, reset
            if(!owlsRobot.getAction().isDelayNeededToCompleteLastAction()) {
                owlsRobot.manualSteer(drive, turn, strafe1, strafe2, powerMultiplier);
            }
        }
    }
    public void initDecodeOp(HardwareMap hardwareMap, Telemetry telemetry, DecodeConstants.TeamAllianceColor teamAllianceColor
            ,DecodeConstants.FieldPosition pStartPosition ,DecodeConstants.FieldPosition pTargetPosition,
                             boolean nearLaunchArea){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.teamAllianceColor = teamAllianceColor;
        this.targetPosition = pTargetPosition;

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
                    heading = DecodeUtil.getLaunchHeading(teamAllianceColor, this.startPose, targetPosition);
                    this.startPose = new Pose(this.startPose.getX(), this.startPose.getY(), heading);
                }
                telemetry.addLine("Start pose set to field position " + pStartPosition.toString());
            } else {
                this.startPose = null;
                telemetry.addLine("Start pose set to 0,0");
            }
        }

        owlsRobot = new DecodeBot(opMode, hardwareMap, telemetry,this.startPose, decodeConfig,
                teamAllianceColor, targetPosition, false);
        telemetry.addLine("Ready to start Driver Op....");
        telemetry.update();
    }
}