package org.firstinspires.ftc.teamcode.decode.bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.AUTO_OP_MODE;

public class ArtifactContainer {
    private ArrayList<DecodeConstants.ArtifactColor> containerSlots = new ArrayList<DecodeConstants.ArtifactColor>(3);
    public Servo pushServo = null;
    public Servo rotateServo = null;
    static final double LINEAR_SERVO_ERROR_TOLERANCE = 0.001;
    static final double ROTATE_SERVO_ERROR_TOLERANCE = 0.001;
    int currentSlot = 0;
    ElapsedTime timeSinceLastRotate = new ElapsedTime();
    ElapsedTime timeSinceLastPush = new ElapsedTime();
    private boolean pushServoAvailable = true,
            rotateServoAvailable= true;

    public double
            pushServoExtendPos = 1,
            pushServoRetractPos = 0,
            rotateServoInitialPos = 0.25,
            rotatePerSlot = 0.25,
            timeNeededtoLaunch = 0.5,
            timeNeededToRotate = 0.5;

    private Telemetry telemetry;
    ArtifactContainer(HardwareMap hardwareMap, DecodeConstants.OP_MODE op_mode, Telemetry telemetry) {
        this.telemetry = telemetry;

        if(AUTO_OP_MODE.equals(op_mode)) {
            InitializeContainerForAuto();
        }

        try {
            pushServo = hardwareMap.get(Servo.class, "pushServo");
            pushServo.setPosition(pushServoRetractPos);
            pushServoAvailable = true;
        } catch (Exception e) {
            pushServoAvailable = false;
            telemetry.addData("Init problem with ", "push servo");
            telemetry.update();
        }

        try {
            rotateServo = hardwareMap.get(Servo.class, "rotateServo");
            rotateServo.setPosition(rotateServoInitialPos);
            rotateServoAvailable = true;
        } catch (Exception e) {
            rotateServoAvailable = false;
            telemetry.addData("Init problem with ", "rotate servo");
            telemetry.update();
        }

        timeSinceLastRotate.reset();
        timeSinceLastPush.reset();
    }
    public void InitializeContainerForAuto() {
        containerSlots.set(0, DecodeConstants.ArtifactColor.PURPLE_ARTIFACT);
        containerSlots.set(1, DecodeConstants.ArtifactColor.GREEN_ARTIFACT);
        containerSlots.set(2, DecodeConstants.ArtifactColor.PURPLE_ARTIFACT);
    }

    public void setContainerToEmpty() {
        containerSlots.set(0, null);
        containerSlots.set(1, null);
        containerSlots.set(2, null);
    }

    public void addArtifactToContainer(int slot, DecodeConstants.ArtifactColor artifactColor) {
        containerSlots.set(slot, artifactColor);
    }

    public void rotateContainer(boolean clockwise){
        if(!rotateServoAvailable) return;

        if(timeSinceLastRotate.time() < timeNeededToRotate) return;

        double targetPosition = 0;
        if(clockwise) {
            currentSlot += 1;
            if(currentSlot > 2) {
                currentSlot = 0;
                targetPosition = getRotatePosition() - 2 * rotatePerSlot;
            } else {
                targetPosition = getRotatePosition() + rotatePerSlot;
            }
        } else {
            currentSlot -= 1;
            if(currentSlot < 0) {
                currentSlot = 2;
                targetPosition = getRotatePosition() + 2 * rotatePerSlot;
            } else {
                targetPosition = getRotatePosition() - rotatePerSlot;
            }
        }

        setRotateServoPosition(targetPosition);
        timeSinceLastRotate.reset();
    }

    public void adjustRotationPerSlot(double adjustment){
        rotatePerSlot += adjustment;
    }

    public void pushArtifactForLaunch() {
        if (!pushServoAvailable) return;

        if(timeSinceLastPush.time() < timeNeededtoLaunch) return;

        ElapsedTime waitClock = new ElapsedTime();

        if(isLinearServoInPushState()) {
            setPushServoToRetractPos();
            waitClock.reset();
            while (waitClock.time() < 0.3) {}
        }

        setLinearServoToExtendPos();
        waitClock.reset();
        while (waitClock.time() < 0.5) {}
        setPushServoToRetractPos();
        pushServo.setPosition(pushServoRetractPos);
        timeSinceLastPush.reset();
    }
    public void setLinearServoToExtendPos(){
        if(!pushServoAvailable) return;

        pushServo.setPosition(pushServoExtendPos);
    }
    public void setPushServoToRetractPos(){
        if(!pushServoAvailable) return;

        pushServo.setPosition(pushServoRetractPos);
    }

    public double getPushServoPosition() {
        return (pushServoAvailable) ? pushServo.getPosition() : -1;
    }
    public boolean isLinearServoInPushState() {
        if(!pushServoAvailable)  return false;

        if (Math.abs(pushServoExtendPos - getPushServoPosition()) <= LINEAR_SERVO_ERROR_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }

    public void setRotateServoPosition(double targetPosition) {
        if(!rotateServoAvailable) return;

        if(isRotatePositionWithinTolerance(targetPosition)) return;

        rotateServo.setPosition(targetPosition);
    }

    public boolean isRotatePositionWithinTolerance(double targetPosition) {
        if (Math.abs(targetPosition - getRotatePosition()) <= ROTATE_SERVO_ERROR_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }
    public double getRotatePosition() {
        return (rotateServo == null) ? -1 : rotateServo.getPosition();
    }

    public String getContainerDisplayInfo() {

        String displayLines = "";

        if(rotateServoAvailable) {
            displayLines += "\n" + "Rotate Servo position = " + getRotatePosition() + " Current Slot = " + currentSlot+1;
        } else {
            displayLines += "\n" + "Servo to rotate container not available!";
        }

        if(pushServoAvailable) {
            displayLines += "\n" + "Push Servo position = " + getPushServoPosition();
        } else {
            displayLines += "\n" + "Servo to push artifact not available!";
        }

        return displayLines;
    }
}