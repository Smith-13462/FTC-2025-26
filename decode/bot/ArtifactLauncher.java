package org.firstinspires.ftc.teamcode.decode.bot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;
import org.firstinspires.ftc.teamcode.decode.common.DecodeUtil;
import org.firstinspires.ftc.teamcode.decode.opmode.AutoOp;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConfig.BLUE_LAUNCH_C1_POSE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConfig.RED_LAUNCH_D1_POSE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_B5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_C1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_C4;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_C5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_D1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_LAUNCH_D5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_NEAR_VERTEX;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_C1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_C4;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_D1;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_D4;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_D5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_LAUNCH_E5;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_NEAR_VERTEX;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.AUTO_OP_MODE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.SpeedUnit.ROTATIONS_PER_MINUTE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.SpeedUnit.TICKS_PER_SECOND;

public class ArtifactLauncher {
    static final double LAUNCH_MOTOR_TICKS_PER_ROTATION = 28 ;
    static final double LAUNCH_MOTOR_RPM = 6000 ;
    static final double LAUNCH_MOTOR_MAX_TICKS_PER_SECOND =
            (LAUNCH_MOTOR_TICKS_PER_ROTATION * LAUNCH_MOTOR_RPM) / 60;
    public DcMotorEx topLaunchMotor = null, bottomLaunchMotor = null;
    private boolean topLaunchMotorAvailable = true, bottomLaunchMotorAvailable = true;
    private Telemetry telemetry;
    private double topMotorLastSetRPM = -1, bottomMotorLastSetRPM = -1;
    private ElapsedTime timeSinceLastSpeedSet;
    private double loadArtifactTopMotorRPM = 1000,  loadArtifactBottomMotorRPM = 1000;
    private double acceptableTopMotorDelta = 25, acceptableBottomMotorDelta = 25;
    private double defaultTopMotorRPM = 500,  defaultBottomMotorRPM = 3500;
    //PIDF tuning to be done
    private double topMotorP = 12.0, topMotorI = 0.0, topMotorD = 2.0, topMotorF = 20000;
    private double bottomMotorP = 12.0, bottomMotorI = 0.0, bottomMotorD = 2.0, bottomMotorF = 20000;
    private DecodeConstants.TeamAllianceColor teamAllianceColor;
    private DecodeConstants.FieldPosition targetPosition;
    private boolean nearLaunchArea;
    private DecodeConstants.OP_MODE op_mode;
    ArtifactLauncher(HardwareMap hardwareMap, DecodeConstants.OP_MODE op_mode, Telemetry telemetry,
                     DecodeConstants.TeamAllianceColor teamAllianceColor,
                     DecodeConstants.FieldPosition targetPosition, boolean nearLaunchArea) {
        this.telemetry = telemetry;
        this.teamAllianceColor = teamAllianceColor;
        this.targetPosition = targetPosition;
        this.nearLaunchArea = nearLaunchArea;
        this.op_mode = op_mode;
        InitializeLauncher(hardwareMap, op_mode);
        timeSinceLastSpeedSet = new ElapsedTime();
        timeSinceLastSpeedSet.reset();
    }
    public void InitializeLauncher(HardwareMap hardwareMap, DecodeConstants.OP_MODE op_mode) {

        try {
            topLaunchMotor = hardwareMap.get(DcMotorEx.class, "toplaunchmotor");
            topLaunchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            topLaunchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            if(AUTO_OP_MODE.equals(op_mode) && nearLaunchArea) {
                topLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                topLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
            }
            topLaunchMotor.setPower(0);

            topLaunchMotorAvailable = true;
        } catch (Exception e) {
            topLaunchMotorAvailable = false;
            telemetry.addData("Init problem with ", "top launch motor");
 //           telemetry.update();
        }


        try {
            bottomLaunchMotor = hardwareMap.get(DcMotorEx.class, "bottomlaunchmotor");
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bottomLaunchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            bottomLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);
            bottomLaunchMotor.setPower(0);

            bottomLaunchMotorAvailable = true;
        } catch (Exception e) {
            bottomLaunchMotorAvailable = false;
            telemetry.addData("Init problem with ", "bottom launch motor");
//            telemetry.update();
        }
    }

    public void adjustTopLaunchVelocityForTest(double velocityAdjustmentFactor) {
        if (velocityAdjustmentFactor > 1) {
            velocityAdjustmentFactor = 1;
        } else if (velocityAdjustmentFactor < 0) {
            velocityAdjustmentFactor = 0;
        }

        double adjustedVelocity = LAUNCH_MOTOR_MAX_TICKS_PER_SECOND * velocityAdjustmentFactor;

        if (topLaunchMotorAvailable) {
            //topLaunchMotor.setVelocityPIDFCoefficients(topMotorP, topMotorI, topMotorD, topMotorF);
            topLaunchMotor.setVelocity(adjustedVelocity);

            topLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            this.topMotorLastSetRPM = convertTPSToRPM(adjustedVelocity);
        }
    }

    public void adjustBottomLaunchVelocityForTest(double velocityAdjustmentFactor) {
        if (velocityAdjustmentFactor > 1) {
            velocityAdjustmentFactor = 1;
        } else if (velocityAdjustmentFactor < 0) {
            velocityAdjustmentFactor = 0;
        }

        double adjustedVelocity = LAUNCH_MOTOR_MAX_TICKS_PER_SECOND * velocityAdjustmentFactor;

        if (bottomLaunchMotorAvailable) {
            if (!isInLaunchMode()) {
                //bottomLaunchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }
            //bottomLaunchMotor.setVelocityPIDFCoefficients(bottomMotorP, bottomMotorI, bottomMotorD, bottomMotorF);
            bottomLaunchMotor.setVelocity(adjustedVelocity);
            bottomLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            this.bottomMotorLastSetRPM = convertTPSToRPM(adjustedVelocity);
        }
    }

    public boolean isAtDesiredSpeedForLaunch(){

        boolean topMotorSpeedWithinDelta = false, bottomMotorSpeedWithinDelta = false;

        if(topLaunchMotorAvailable) {
            if (((getTopLaunchVelocity(ROTATIONS_PER_MINUTE) >= (topMotorLastSetRPM - acceptableTopMotorDelta)) &&
                    (getTopLaunchVelocity(ROTATIONS_PER_MINUTE) <= topMotorLastSetRPM + acceptableTopMotorDelta))) {
                topMotorSpeedWithinDelta = true;
            }
        } else {
            topMotorSpeedWithinDelta = true;
        }

        if(bottomLaunchMotorAvailable) {
            if (((getBottomLaunchVelocity(ROTATIONS_PER_MINUTE) >= (bottomMotorLastSetRPM - acceptableBottomMotorDelta)) &&
                    (getBottomLaunchVelocity(ROTATIONS_PER_MINUTE) <= bottomMotorLastSetRPM + acceptableBottomMotorDelta))) {
                bottomMotorSpeedWithinDelta = true;
            }
        } else {
            bottomMotorSpeedWithinDelta = true;
        }

        if(topMotorSpeedWithinDelta && bottomMotorSpeedWithinDelta) {
            return true;
        } else {
            return false;
        }
    }

    public double getTopLaunchVelocity(DecodeConstants.SpeedUnit unit) {

        if (topLaunchMotorAvailable) {
            if(TICKS_PER_SECOND.equals(unit)) {
                return  topLaunchMotor.getVelocity();
            } else {
              return convertTPSToRPM(topLaunchMotor.getVelocity());
            }
        }

        return 0;
    }

    public double getBottomLaunchVelocity(DecodeConstants.SpeedUnit unit) {

        if (bottomLaunchMotorAvailable) {
            if(TICKS_PER_SECOND.equals(unit)) {
                return bottomLaunchMotor.getVelocity();
            } else {
                return convertTPSToRPM(bottomLaunchMotor.getVelocity());
            }
        }

        return 0;
    }

    public boolean adjustLaunchVelocity(double requestedTopMotorRPM, double requestedBottomMotorRPM){
        boolean anyAdjustmentDone = false;

        double topMotorVelocity = convertRPMtoTPS(requestedTopMotorRPM);
        double bottomMotorVelocity = convertRPMtoTPS(requestedBottomMotorRPM);

        topMotorVelocity = Range.clip(topMotorVelocity, 0, LAUNCH_MOTOR_MAX_TICKS_PER_SECOND);
        bottomMotorVelocity = Range.clip(bottomMotorVelocity, 0, LAUNCH_MOTOR_MAX_TICKS_PER_SECOND);

        boolean resetEncoder = !isInLaunchMode();

        if (topLaunchMotorAvailable && (requestedTopMotorRPM != topMotorLastSetRPM)) {
            //topLaunchMotor.setVelocityPIDFCoefficients(topMotorP, topMotorI, topMotorD, topMotorF);
            topLaunchMotor.setVelocity(topMotorVelocity);
            if(AUTO_OP_MODE.equals(op_mode) && nearLaunchArea) {
                topLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                topLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
            }
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.topMotorLastSetRPM = requestedTopMotorRPM;
            anyAdjustmentDone = true;
        }

        if (bottomLaunchMotorAvailable && (requestedTopMotorRPM != bottomMotorLastSetRPM)) {
            if (resetEncoder) {
                //bottomLaunchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }
            //bottomLaunchMotor.setVelocityPIDFCoefficients(bottomMotorP, bottomMotorI, bottomMotorD, bottomMotorF);
            bottomLaunchMotor.setVelocity(bottomMotorVelocity);
            bottomLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.bottomMotorLastSetRPM = requestedBottomMotorRPM;
            anyAdjustmentDone = true;
        }

        return anyAdjustmentDone;
    }

    public void setDefaultPropulsionSpeed(){
        double topMotorVelocity = convertRPMtoTPS(defaultTopMotorRPM);
        double bottomMotorVelocity = convertRPMtoTPS(defaultBottomMotorRPM);

        topMotorVelocity = Range.clip(topMotorVelocity, 0, LAUNCH_MOTOR_MAX_TICKS_PER_SECOND);
        bottomMotorVelocity = Range.clip(bottomMotorVelocity, 0, LAUNCH_MOTOR_MAX_TICKS_PER_SECOND);

        if (topLaunchMotorAvailable) {
            topLaunchMotor.setPower(0);
            topLaunchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //topLaunchMotor.setVelocityPIDFCoefficients(topMotorP, topMotorI, topMotorD, topMotorF);
            topLaunchMotor.setVelocity(topMotorVelocity);
            if(AUTO_OP_MODE.equals(op_mode) && nearLaunchArea) {
                topLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                topLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
            }
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.topMotorLastSetRPM = loadArtifactTopMotorRPM;
        }

        if (bottomLaunchMotorAvailable) {
            bottomLaunchMotor.setPower(0);
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
           //bottomLaunchMotor.setVelocityPIDFCoefficients(bottomMotorP, bottomMotorI, bottomMotorD, bottomMotorF);
            bottomLaunchMotor.setVelocity(bottomMotorVelocity);
            bottomLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.bottomMotorLastSetRPM = loadArtifactBottomMotorRPM;
        }
    }

    public void setLoadArtifactsSpeed(){
        double topMotorVelocity = convertRPMtoTPS(loadArtifactTopMotorRPM);
        double bottomMotorVelocity = convertRPMtoTPS(loadArtifactBottomMotorRPM);

        topMotorVelocity = Range.clip(topMotorVelocity, 0, LAUNCH_MOTOR_MAX_TICKS_PER_SECOND);
        bottomMotorVelocity = Range.clip(bottomMotorVelocity, 0, LAUNCH_MOTOR_MAX_TICKS_PER_SECOND);

        turnOffLaunchPower();

        if (topLaunchMotorAvailable) {
            topLaunchMotor.setPower(0);
            topLaunchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //topLaunchMotor.setVelocityPIDFCoefficients(topMotorP, topMotorI, topMotorD, topMotorF);
            topLaunchMotor.setVelocity(topMotorVelocity);
            topLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.topMotorLastSetRPM = loadArtifactTopMotorRPM;
        }

        if (bottomLaunchMotorAvailable) {
            bottomLaunchMotor.setPower(0);
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //bottomLaunchMotor.setVelocityPIDFCoefficients(bottomMotorP, bottomMotorI, bottomMotorD, bottomMotorF);
            bottomLaunchMotor.setVelocity(bottomMotorVelocity);
            bottomLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.bottomMotorLastSetRPM = loadArtifactBottomMotorRPM;
        }
    }

    public boolean setPropulsionSpeed(Pose launchPoint){
        boolean anyVelocityAdjustmentDone = false;

        double topLaunchSpeed = 1400;
        double bottomLaunchSpeed = 1400;

        DecodeConstants.FieldPosition launchPositionName = DecodeUtil.getLaunchPositionName(launchPoint, teamAllianceColor);

        if(launchPositionName != null){
            setPropulsionSpeed(launchPositionName);
        } else {
            double diagonalDistance = DecodeUtil.getDiagonalDistanceToTarget(teamAllianceColor, launchPoint, targetPosition);
            topLaunchSpeed = 1400 + (500 * (diagonalDistance / 120));
            bottomLaunchSpeed = 1400 + (800 * (diagonalDistance / 120)) - (300 * (diagonalDistance / 120));

            anyVelocityAdjustmentDone = adjustLaunchVelocity(topLaunchSpeed, bottomLaunchSpeed);
        }


        return anyVelocityAdjustmentDone;
    }

    public boolean setPropulsionSpeed(DecodeConstants.FieldPosition launchPosition) {
        boolean anyVelocityAdjustmentDone = false;

        if(RED_LAUNCH_D1.equals(launchPosition)){
            anyVelocityAdjustmentDone =  adjustLaunchVelocity(520, 3470);
        } else if (RED_LAUNCH_C1.equals(launchPosition)){
            anyVelocityAdjustmentDone =  adjustLaunchVelocity(500, 3700);
        } else if (BLUE_LAUNCH_C1.equals(launchPosition)){
            anyVelocityAdjustmentDone =  adjustLaunchVelocity(520, 3520);
        } else if (BLUE_LAUNCH_D1.equals(launchPosition)){
            anyVelocityAdjustmentDone =  adjustLaunchVelocity(500, 3600);
        } else if(RED_LAUNCH_D5.equals(launchPosition) ||
                BLUE_LAUNCH_C5.equals(launchPosition)) {
            if(AUTO_OP_MODE.equals(op_mode)){
                anyVelocityAdjustmentDone =  adjustLaunchVelocity(500, 3600);
            } else {
                anyVelocityAdjustmentDone = adjustLaunchVelocity(1000, 2350);
            }
        } else if(RED_LAUNCH_E5.equals(launchPosition) ||
                BLUE_LAUNCH_B5.equals(launchPosition)) {
            anyVelocityAdjustmentDone =  adjustLaunchVelocity(300, 3300);
        } else if(RED_LAUNCH_D4.equals(launchPosition) ||
                BLUE_LAUNCH_C4.equals(launchPosition)) {
            anyVelocityAdjustmentDone =  adjustLaunchVelocity( 600,2950);
        } else if(RED_LAUNCH_C4.equals(launchPosition) ||
                BLUE_LAUNCH_D5.equals(launchPosition)) {
            anyVelocityAdjustmentDone =  adjustLaunchVelocity( 600,3130);
        } else if(RED_NEAR_VERTEX.equals(launchPosition) ||
                BLUE_NEAR_VERTEX.equals(launchPosition)) {
            anyVelocityAdjustmentDone =  adjustLaunchVelocity(600, 3050);
        }


        return anyVelocityAdjustmentDone;
    }
    public void turnOffLaunchPower() {
        if(topLaunchMotorAvailable) {
            topLaunchMotor.setPower(0);
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            topMotorLastSetRPM = 0;
        }

        if(bottomLaunchMotorAvailable) {
            bottomLaunchMotor.setPower(0);
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            bottomMotorLastSetRPM = 0;
        }
    }

    public void reverseTopMotorDirection(boolean forward) {
        if(topLaunchMotorAvailable) {
            double topMotorVelocity = convertRPMtoTPS(500);
            if(forward) {
                topLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
            }else {
                topLaunchMotor.setDirection(DcMotorEx.Direction.REVERSE);
            }
            topLaunchMotor.setVelocity(topMotorVelocity);
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            topMotorLastSetRPM = 500;
        }
    }

    public boolean isInLoadingArtifactsMode(){
        if((topLaunchMotorAvailable && (topMotorLastSetRPM > 0) && DcMotorEx.Direction.REVERSE.equals(topLaunchMotor.getDirection())) ||
                (bottomLaunchMotorAvailable && (bottomMotorLastSetRPM > 0) && DcMotorEx.Direction.FORWARD.equals(bottomLaunchMotor.getDirection()))){
            return true;
        } else {
            return false;
        }
    }

    public boolean isInLaunchMode(){
        if((topLaunchMotorAvailable && (topMotorLastSetRPM > 0) && DcMotorEx.Direction.FORWARD.equals(topLaunchMotor.getDirection())) ||
                (bottomLaunchMotorAvailable && (bottomMotorLastSetRPM > 0) && DcMotorEx.Direction.REVERSE.equals(bottomLaunchMotor.getDirection()))){
            return true;
        } else {
            return false;
        }
    }

    public String getLauncherDisplayInfo() {

        String displayLines = "";

        displayLines += "\n" + "Top launch speed: " + String.format("%6.1f", getTopLaunchVelocity(ROTATIONS_PER_MINUTE)) + " RPM (TPS: "
                + String.format("%6.1f", getTopLaunchVelocity(TICKS_PER_SECOND)) + ")";

        displayLines += "\n" + "Bottom launch speed: " + String.format("%6.1f", getBottomLaunchVelocity(ROTATIONS_PER_MINUTE)) + " RPM (TPS: "
                + String.format("%6.1f", getBottomLaunchVelocity(TICKS_PER_SECOND)) + ")";

        return displayLines;
    }

    private double convertTPSToRPM(double motorSpeedTPS){
        return (motorSpeedTPS * 60) / LAUNCH_MOTOR_TICKS_PER_ROTATION;
    }

    private double convertRPMtoTPS(double motorSpeedRPM){
       return  (motorSpeedRPM * LAUNCH_MOTOR_TICKS_PER_ROTATION) / 60;
    }
}