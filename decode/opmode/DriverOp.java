package org.firstinspires.ftc.teamcode.decode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.bot.DecodeBot;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.DRIVER_OP_MODE;

@TeleOp(name="Basic:ZZZDoNOTUseCommonDriverOp", group="DriverOp")
//@Disabled
public abstract class DriverOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timeSinceAutoPosRequest = new ElapsedTime();
    public DecodeBot owlsRobot;
    double powerMultiplier = 0.7;
    private Pose startPose;
    boolean autoPositionState = false;
    double powerAdjustment = 0.5, velocityAdjustment =0.5, angleAdjustment=55;
    private DecodeConstants.OP_MODE opMode = DRIVER_OP_MODE;
    DecodeConstants.TeamAllianceColor teamAllianceColor = null;
    @Override
    public void runOpMode() {
        waitForStart();
        runtime.reset();
        this.autoPositionState = false;
        timeSinceAutoPosRequest.reset();
        telemetry.addLine("Driver Op in progress..");
        telemetry.update();

        while (opModeIsActive())
        {
            //////////////////////Driver 1 (gamepad 1) request//////////////////
            //movement related
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe1 = gamepad1.left_stick_x;
//            double strafe1 = gamepad1.left_trigger;
            double strafe2 = gamepad1.right_trigger;
            if (gamepad1.a ){
                powerMultiplier = powerMultiplier + 0.05;
                if (powerMultiplier > 0.8) powerMultiplier = 0.8;
            } else if (gamepad1.b) {
                powerMultiplier = powerMultiplier - 0.05;
                if (powerMultiplier < 0.4) powerMultiplier = 0.4;
            }

            if(gamepad1.dpad_up) {
                if(!isDelayNeeded()) {
                    this.autoPositionState = true;
                    timeSinceAutoPosRequest.reset();
                    owlsRobot.getMotion().turnToLaunch(teamAllianceColor);
                }
            } else if(gamepad1.dpad_down) {
                owlsRobot.getMotion().turnOffPositionHold(teamAllianceColor);
                this.autoPositionState = false;
            }

            if(gamepad2.x) {
                powerAdjustment += 0.05;
                if(powerAdjustment > 1) { powerAdjustment = 1; }
                owlsRobot.getLauncher().adjustLaunchPower(powerAdjustment);
            }  else if(gamepad2.y) {
                powerAdjustment -= 0.05;
                if(powerAdjustment < 0) { powerAdjustment = 0; }
                owlsRobot.getLauncher().adjustLaunchPower(powerAdjustment);
            } else if (gamepad2.a) {
                velocityAdjustment += 0.05;
                if(velocityAdjustment < 0) { velocityAdjustment = 0; }
                owlsRobot.getLauncher().adjustLaunchVelocity(velocityAdjustment);
            } else if(gamepad2.b) {
                velocityAdjustment -= 0.05;
                if(velocityAdjustment < 0) { velocityAdjustment = 0; }
                owlsRobot.getLauncher().adjustLaunchVelocity(velocityAdjustment);
            }

            if(gamepad2.dpad_up) {
                angleAdjustment += 1;
                if(angleAdjustment > 70) { angleAdjustment = 70; }
                owlsRobot.getLauncher().adjustLaunchAngle(angleAdjustment);
            } else if (gamepad2.dpad_down) {
                angleAdjustment -= 1;
                if(angleAdjustment < 40) { angleAdjustment = 40; }
                owlsRobot.getLauncher().adjustLaunchAngle(angleAdjustment);
            }

            if(gamepad2.dpad_left) {
                owlsRobot.getContainer().adjustRotationPerSlot(-0.01);
            } else if (gamepad2.dpad_right) {
                owlsRobot.getContainer().adjustRotationPerSlot(0.01);
            }

            if(gamepad2.left_bumper){
                owlsRobot.getContainer().rotateContainer(false);
            } else if(gamepad2.right_bumper){
                owlsRobot.getContainer().rotateContainer(true);
            }

            if(gamepad2.right_trigger > 0.1) {
                owlsRobot.getContainer().pushArtifactForLaunch();
            }

            telemetry.addLine(owlsRobot.getLauncher().getLauncherDisplayInfo());
            telemetry.addLine(owlsRobot.getContainer().getContainerDisplayInfo());
            telemetry.addLine(owlsRobot.getMotion().getBotPositionDisplayInfo(teamAllianceColor, true));
            telemetry.update();

            //driver 1 movement, reset
            if(!isDelayNeeded()) {
                owlsRobot.manualSteer(drive, turn, strafe1, strafe2, powerMultiplier);
            }
        }
    }
    public void initDecodeOp(HardwareMap hardwareMap, Telemetry telemetry, DecodeConstants.TeamAllianceColor teamAllianceColor, Pose pStartPose){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.teamAllianceColor = teamAllianceColor;
        this.startPose = pStartPose;
        this.autoPositionState = false;

        owlsRobot = new DecodeBot(opMode, hardwareMap, telemetry,this.startPose);
    }

    private boolean isDelayNeeded() {
        boolean delayNeeded = false;
        if(this.autoPositionState && (timeSinceAutoPosRequest.time() < 2)) {
            delayNeeded = true;
        }

        return delayNeeded;
    }
}