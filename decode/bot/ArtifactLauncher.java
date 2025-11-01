package org.firstinspires.ftc.teamcode.decode.bot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.GREEN_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.PURPLE_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Motif.PGP_MOTIF;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Motif.UNKNOWN_MOTIF;

public class ArtifactLauncher {
    static final double LAUNCH_MOTOR_MAX_SPEED = 1.0;
    static final double LAUNCH_MOTOR_TICKS_PER_ROTATION = 28 ;
    static final double LAUNCH_MOTOR_RPM = 6000 ;
    static final double LAUNCH_MOTOR_MAX_TICKS_PER_SECOND =
            (LAUNCH_MOTOR_TICKS_PER_ROTATION * LAUNCH_MOTOR_RPM) / 60;
    public DcMotorEx topLaunchMotor = null, bottomLaunchMotor = null;
    private boolean topLaunchMotorAvailable = true, bottomLaunchMotorAvailable = true;
    public Servo angleServo = null;
    private boolean angleServoAvailable = true;
    public double lastKnownAngleServoPos = -1;
    public double angleServoInitialPos = 0.5;
    public double timeNeededToAdjustAngle = 1;
    public double timeNeededToAdjustSpeed = 1;
    static final double LAUNCH_ANGLE_ERROR_TOLERANCE = 0.1;
    private Boolean velocityAdjustmentMode = null;
    ElapsedTime timeSinceLastAngleAdjustment = new ElapsedTime();
    ElapsedTime timeSinceLastSpeedAdjustment = new ElapsedTime();


    private double currentPower = 0, currentTopMotorTicksPerSecond = 0, currentBottomMotorTicksPerSecond;
    private Telemetry telemetry;
    private double MINIMUM_LAUNCH_ANGLE =40, MAXIMUM_LAUNCH_ANGLE = 70;
    private double MINIMUM_LAUNCH_ANGLE_SERVO_POSITION =0, MAXIMUM_LAUNCH_ANGLE_SERVO_POSITION = 1;
    ArtifactLauncher(HardwareMap hardwareMap, DecodeConstants.OP_MODE op_mode, Telemetry telemetry) {
        this.telemetry = telemetry;
        InitializeLauncher(hardwareMap, op_mode);
    }
    public void InitializeLauncher(HardwareMap hardwareMap, DecodeConstants.OP_MODE op_mode) {

        timeSinceLastSpeedAdjustment.reset();
        timeSinceLastAngleAdjustment.reset();

        try {
            topLaunchMotor = hardwareMap.get(DcMotorEx.class, "toplaunchmotor");
            topLaunchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            topLaunchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            topLaunchMotor.setDirection(DcMotorEx.Direction.FORWARD);
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
/*
        try {
            angleServo = hardwareMap.get(Servo.class, "angleservo");
            angleServo.setPosition(angleServoInitialPos);
            angleServoAvailable = true;
        } catch (Exception e) {
            angleServoAvailable = false;
            telemetry.addData("Init problem with ", "angle servo");
            telemetry.update();
        }
 */
    }

    /*
    public void adjustLaunchPower(double speedAdjustmentFactor) {
        if(timeSinceLastSpeedAdjustment.time() < timeNeededToAdjustSpeed) return;

        if (speedAdjustmentFactor > 1) {
            speedAdjustmentFactor = 1;
        } else if (speedAdjustmentFactor < 0) {
            speedAdjustmentFactor = 0;
        }

        double adjustedPower = LAUNCH_MOTOR_MAX_SPEED * speedAdjustmentFactor;

        if((velocityAdjustmentMode != null) && (velocityAdjustmentMode == true)) {
            turnOffLaunchPower();
        }
        velocityAdjustmentMode = false;

        if (launchMotorsAvailable) {
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            topLaunchMotor.setPower(adjustedPower);

            bottomLaunchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bottomLaunchMotor.setPower(adjustedPower);

            this.currentPower = adjustedPower;
        }

        timeSinceLastSpeedAdjustment.reset();
    }
*/
    public void adjustTopLaunchVelocity(double velocityAdjustmentFactor) {
        if (velocityAdjustmentFactor > 1) {
            velocityAdjustmentFactor = 1;
        } else if (velocityAdjustmentFactor < 0) {
            velocityAdjustmentFactor = 0;
        }

        double adjustedVelocity = LAUNCH_MOTOR_MAX_TICKS_PER_SECOND * velocityAdjustmentFactor;

        if (topLaunchMotorAvailable) {
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            topLaunchMotor.setVelocity(adjustedVelocity);;

            this.currentTopMotorTicksPerSecond = adjustedVelocity;
        }
    }

    public void adjustBottomLaunchVelocity(double velocityAdjustmentFactor) {
        if (velocityAdjustmentFactor > 1) {
            velocityAdjustmentFactor = 1;
        } else if (velocityAdjustmentFactor < 0) {
            velocityAdjustmentFactor = 0;
        }

        double adjustedVelocity = LAUNCH_MOTOR_MAX_TICKS_PER_SECOND * velocityAdjustmentFactor;

        if (bottomLaunchMotorAvailable) {
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bottomLaunchMotor.setVelocity(adjustedVelocity);

            this.currentBottomMotorTicksPerSecond = adjustedVelocity;
        }
    }

    private void turnOffLaunchPower() {
        if(topLaunchMotorAvailable) {
            topLaunchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            topLaunchMotor.setPower(0);
        }

        if(bottomLaunchMotorAvailable) {
            bottomLaunchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bottomLaunchMotor.setPower(0);
        }
    }
    public double getServoPositionForAngle(double angle) {
       double position = 0;

        angle = Range.clip(angle,MINIMUM_LAUNCH_ANGLE,  MAXIMUM_LAUNCH_ANGLE);

        position =  (angle - MINIMUM_LAUNCH_ANGLE) /
                (MAXIMUM_LAUNCH_ANGLE - MINIMUM_LAUNCH_ANGLE);

       return position;
    }
    public double getAngleOfServoPosition(double position) {
        double angle = 0;

        position = Range.clip(position,MINIMUM_LAUNCH_ANGLE_SERVO_POSITION,  MAXIMUM_LAUNCH_ANGLE_SERVO_POSITION);

        angle = MINIMUM_LAUNCH_ANGLE + (((position - MINIMUM_LAUNCH_ANGLE_SERVO_POSITION) /
                (MAXIMUM_LAUNCH_ANGLE_SERVO_POSITION - MINIMUM_LAUNCH_ANGLE_SERVO_POSITION))
                * (MAXIMUM_LAUNCH_ANGLE - MINIMUM_LAUNCH_ANGLE));

        return angle;
    }
    public void setServoPosition(double position) {
        if (!angleServoAvailable) return;

        position = Range.clip(position,MINIMUM_LAUNCH_ANGLE_SERVO_POSITION,  MAXIMUM_LAUNCH_ANGLE_SERVO_POSITION);

        angleServo.setPosition(position);
        lastKnownAngleServoPos = position;
    }

    /*
    public void adjustLaunchAngle(double targetAngle) {
        if (!angleServoAvailable) return;

        if(timeSinceLastAngleAdjustment.time() < timeNeededToAdjustAngle) return;

        if(isLaunchAngleWithinTolerance(targetAngle)) return;

        double targetPosition = 0;
        targetPosition = getServoPositionForAngle(targetAngle);
        setServoPosition(targetPosition);
        timeSinceLastAngleAdjustment.reset();

        lastKnownAngleServoPos = targetPosition;
    }


    public boolean isLaunchAngleWithinTolerance(double targetAngle) {
        double currentAngle = getCurrentLaunchAngle();

        if (Math.abs(targetAngle - currentAngle) <= LAUNCH_ANGLE_ERROR_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }

    public double getCurrentLaunchAngleServoPosition() {
        return (angleServoAvailable) ? angleServo.getPosition() : -1;
    }
    public double getCurrentLaunchAngle() {
        if(!angleServoAvailable) return -1;

        return getAngleOfServoPosition(getCurrentLaunchAngleServoPosition());
    }
*/
    public String getLauncherDisplayInfo() {

        String displayLines = "";

        displayLines += "\n" + "Using " + String.format("%6.1f", currentTopMotorTicksPerSecond) + " ticks per second(RPM "
                + String.format("%6.1f", (currentTopMotorTicksPerSecond * 60) / LAUNCH_MOTOR_TICKS_PER_ROTATION)
                + " for top motor to launch";
        displayLines += "\n" + "Using " + String.format("%6.1f", currentBottomMotorTicksPerSecond) + " ticks per second(RPM "
                + String.format("%6.1f", (currentBottomMotorTicksPerSecond * 60) / LAUNCH_MOTOR_TICKS_PER_ROTATION)
                + " for bottom motor to launch";
        return displayLines;
    }
}