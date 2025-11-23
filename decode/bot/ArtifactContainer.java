package org.firstinspires.ftc.teamcode.decode.bot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.GREEN_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.NO_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.PURPLE_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.UNKNOWN_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.AUTO_OP_MODE;

public class ArtifactContainer {
    private ArrayList<DecodeConstants.ArtifactColor> containerSlots;
    private ArrayList<Double> rotateServoPositions;
    public Servo pushServo = null;
    public Servo rotateServo = null;
    int currentPickupSlotPositionIndex = 2;
    int currentPushSlotPositionIndex = 0;
    int currentMiddleSlotPositionIndex = 1;
    private final int CONTAINER_SIZE = 3;
    ElapsedTime timeSinceLastRotate = new ElapsedTime();
    ElapsedTime timeSinceLastPush = new ElapsedTime();
    private boolean pushServoAvailable = true, rotateServoAvailable= true;
    ArrayList<NormalizedColorSensor> colorSensors;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ArrayList<Boolean> colorSensorsAvailable;
    private ArrayList<String> hsvDistances = new ArrayList<String>();

    public double
            pushServoExtendPos = 0.15,
            pushServoRetractPos = 0.5,
            rotateServoInitialPos = 0.05,
            timeNeededToPush = 1.25,
            timeNeededToRotate = 0.30;

    ArtifactContainer(HardwareMap hardwareMap, DecodeConstants.OP_MODE op_mode, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initColorSensors();

        if(AUTO_OP_MODE.equals(op_mode)) {
            initializeContainerWithPreloaded();
        } else {
            initializeContainer();
        }
        setPreDefinedServoPositions();

        try {
            pushServo = hardwareMap.get(Servo.class, "pushservo");
            pushServo.setPosition(pushServoRetractPos);
            pushServoAvailable = true;
        } catch (Exception e) {
            pushServoAvailable = false;
            telemetry.addData("Init problem with ", "push servo");
        }

        try {
            rotateServo = hardwareMap.get(Servo.class, "rotateservo");
            rotateServo.setPosition(rotateServoInitialPos);
            rotateServoAvailable = true;
        } catch (Exception e) {
            rotateServoAvailable = false;
            telemetry.addData("Init problem with ", "rotate servo");
        }

        timeSinceLastRotate.reset();
        timeSinceLastPush.reset();
    }
    private void initializeContainerWithPreloaded() {
        containerSlots = new ArrayList<>();
        containerSlots.add(0, PURPLE_ARTIFACT);
        containerSlots.add(1, GREEN_ARTIFACT);
        containerSlots.add(2, PURPLE_ARTIFACT);
    }
    private void initializeContainer() {
        containerSlots = new ArrayList<>();

        for(int slotIdx = 0; slotIdx < CONTAINER_SIZE; slotIdx++){
            containerSlots.add(slotIdx, detectArtifactColor(slotIdx));
        }
    }

    private void setPreDefinedServoPositions(){
        rotateServoPositions = new ArrayList<>();
        rotateServoPositions.add(0, 0.05);
        rotateServoPositions.add(1, 0.5);
        rotateServoPositions.add(2, 0.95);
    }
    private void initColorSensors() {
        float gain = 2;
        colorSensors = new ArrayList<>();
        colorSensorsAvailable = new ArrayList<>();

        for(Integer colorSensorIdx=0; colorSensorIdx<CONTAINER_SIZE; colorSensorIdx++) {
            try {
                Integer tempIdx = colorSensorIdx + 1;
                colorSensors.add(colorSensorIdx,
                        hardwareMap.get(NormalizedColorSensor.class, "colorsensor" + tempIdx.toString()));

                if (colorSensors.get(colorSensorIdx) instanceof SwitchableLight) {
                    ((SwitchableLight) colorSensors.get(colorSensorIdx)).enableLight(true);
                }
                colorSensors.get(colorSensorIdx).setGain(gain);

                if (colorSensors.get(colorSensorIdx) instanceof SwitchableLight) {
                    SwitchableLight light = (SwitchableLight) colorSensors.get(colorSensorIdx);
                    light.enableLight(!light.isLightOn());
                }

                colorSensorsAvailable.add(colorSensorIdx.intValue(), Boolean.TRUE);

                hsvDistances.add(colorSensorIdx.intValue(), new String(""));

            } catch (Exception e) {
                colorSensorsAvailable.add(colorSensorIdx, Boolean.FALSE);
            }
        }
    }

    public boolean alignToLaunchSpecificArtifact(DecodeConstants.ArtifactColor desiredArtifactColor, boolean
                                              waitUntilRotationIsComplete) {
        boolean rotationDone = false;

        if (!rotateServoAvailable) return rotationDone;

        if (desiredArtifactColor == null) return rotationDone;

        DecodeConstants.ArtifactColor launchSlotArtifactColor = getLaunchSlotArtifactColor();
        DecodeConstants.ArtifactColor middleSlotArtifactColor = getMiddleSlotArtifactColor();
        DecodeConstants.ArtifactColor pickupSlotArtifactColor = getPickupSlotArtifactColor();

        if(desiredArtifactColor.equals(launchSlotArtifactColor)) {
            //do nothing
        } else if(desiredArtifactColor.equals(middleSlotArtifactColor)) {
            rotateContainer(true, false);
            rotationDone = true;
            timeSinceLastRotate.reset();
        } else if(desiredArtifactColor.equals(pickupSlotArtifactColor)) {
            rotateContainer(false, false);
            rotationDone = true;
            timeSinceLastRotate.reset();
        }
        return rotationDone;
    }

    public boolean isSpecificArtifactInContainer(DecodeConstants.ArtifactColor desiredArtifactColor){
        boolean artifactInContainer = false;

        if (!rotateServoAvailable) return artifactInContainer;

        if (desiredArtifactColor == null) return artifactInContainer;

        DecodeConstants.ArtifactColor launchSlotArtifactColor = getLaunchSlotArtifactColor();
        DecodeConstants.ArtifactColor middleSlotArtifactColor = getMiddleSlotArtifactColor();
        DecodeConstants.ArtifactColor pickupSlotArtifactColor = getPickupSlotArtifactColor();

        if(desiredArtifactColor.equals(launchSlotArtifactColor) ||
                desiredArtifactColor.equals(middleSlotArtifactColor) ||
                        desiredArtifactColor.equals(pickupSlotArtifactColor)) {
            artifactInContainer = true;
        }

        return artifactInContainer;

    }

    public boolean isLaunchSlotFilledWithArtifact(){
        DecodeConstants.ArtifactColor launchSlotArtifactColor = getLaunchSlotArtifactColor();

        if(PURPLE_ARTIFACT.equals(launchSlotArtifactColor) || GREEN_ARTIFACT.equals(launchSlotArtifactColor)){
           return true;
        } else {
            return false;
        }
    }

    public int getNumberOfEmptySlots(){
        int emptySlots = 0;

        if (!rotateServoAvailable) return emptySlots;

        DecodeConstants.ArtifactColor launchSlotArtifactColor = getLaunchSlotArtifactColor();
        DecodeConstants.ArtifactColor middleSlotArtifactColor = getMiddleSlotArtifactColor();
        DecodeConstants.ArtifactColor pickupSlotArtifactColor = getPickupSlotArtifactColor();

        if(NO_ARTIFACT.equals(launchSlotArtifactColor)){
            emptySlots++;
        }

        if(NO_ARTIFACT.equals(middleSlotArtifactColor)){
            emptySlots++;
        }
        if(NO_ARTIFACT.equals(pickupSlotArtifactColor)){
            emptySlots++;
        }
        return emptySlots;
    }

    public void rotateContainer(boolean clockwise, boolean waitUntilRotationIsComplete){
        if(!rotateServoAvailable) return;

        if(timeSinceLastRotate.time() < timeNeededToRotate) return;

        if(clockwise) {
            currentPickupSlotPositionIndex = (currentPickupSlotPositionIndex + 1 + CONTAINER_SIZE) % CONTAINER_SIZE;
            currentPushSlotPositionIndex = (currentPushSlotPositionIndex + 1 + CONTAINER_SIZE) % CONTAINER_SIZE;
           currentMiddleSlotPositionIndex = (currentMiddleSlotPositionIndex + 1 + CONTAINER_SIZE) % CONTAINER_SIZE;
        } else {
            currentPickupSlotPositionIndex = (currentPickupSlotPositionIndex - 1 + CONTAINER_SIZE) % CONTAINER_SIZE;
            currentPushSlotPositionIndex = (currentPushSlotPositionIndex - 1 + CONTAINER_SIZE) % CONTAINER_SIZE;
            currentMiddleSlotPositionIndex = (currentMiddleSlotPositionIndex - 1 + CONTAINER_SIZE) % CONTAINER_SIZE;
        }

        double targetPosition;
        targetPosition= rotateServoPositions.get(currentPushSlotPositionIndex);
        setRotateServoPosition(targetPosition);
        timeSinceLastRotate.reset();
        while(waitUntilRotationIsComplete && (timeSinceLastRotate.time() < timeNeededToRotate)){}
    }

    public boolean rotateToAlignSlotForLaunch(int rotateToSlot, boolean waitUntilRotationIsComplete){
        boolean rotateDone = false;
        if(!rotateServoAvailable) return rotateDone;

        if(timeSinceLastRotate.time() < timeNeededToRotate) return rotateDone;

        int slotsToRotate = (rotateToSlot - currentPushSlotPositionIndex + CONTAINER_SIZE) % CONTAINER_SIZE;

        if(Math.abs(slotsToRotate) > 0){
            currentPickupSlotPositionIndex = (currentPickupSlotPositionIndex + slotsToRotate + CONTAINER_SIZE) % CONTAINER_SIZE;
            currentPushSlotPositionIndex = (currentPushSlotPositionIndex + slotsToRotate + CONTAINER_SIZE) % CONTAINER_SIZE;
            currentMiddleSlotPositionIndex = (currentMiddleSlotPositionIndex + slotsToRotate + CONTAINER_SIZE) % CONTAINER_SIZE;
            rotateDone = true;

            double targetPosition;
            targetPosition= rotateServoPositions.get(currentPushSlotPositionIndex);
            setRotateServoPosition(targetPosition);
            timeSinceLastRotate.reset();
            while(waitUntilRotationIsComplete && (timeSinceLastRotate.time() < timeNeededToRotate)){}
        }

        return rotateDone;
    }

    public int getLaunchSlotPositionIndex() {
        return currentPushSlotPositionIndex;
    }

    public int getPickupSlotPositionIndex() {
        return currentPickupSlotPositionIndex;
    }

    public int getMiddleSlotPositionIndex() {
        return currentMiddleSlotPositionIndex;

    }

    public DecodeConstants.ArtifactColor detectArtifactColor(int colorSensorIdx) {
        DecodeConstants.ArtifactColor artifactColor = DecodeConstants.ArtifactColor.UNKNOWN_ARTIFACT;

        if(colorSensorsAvailable.get(colorSensorIdx) == Boolean.FALSE){
            return artifactColor;
        }

        try {
            final float[] hsvValues = new float[3];
            NormalizedRGBA colors = colorSensors.get(colorSensorIdx).getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            double distance = 0;
            if (colorSensors.get(colorSensorIdx) instanceof DistanceSensor) {
                distance = ((DistanceSensor) colorSensors.get(colorSensorIdx)).getDistance(DistanceUnit.MM);
                hsvDistances.set(colorSensorIdx, new String(String.format("%6.3f", distance) + "," + hsvValues[0] +","+ hsvValues[1]));
            }

            if ((hsvValues[0] >= 100) && (hsvValues[0] <= 175) && (hsvValues[1] >= 0.7 || (distance <= 80))) {
                artifactColor = DecodeConstants.ArtifactColor.GREEN_ARTIFACT;
            } else if ((hsvValues[0] >= 200) && (hsvValues[0] <= 250) && ((hsvValues[1] >= 0.4) || (distance <= 80))) {
                artifactColor = PURPLE_ARTIFACT;
            }

            if(UNKNOWN_ARTIFACT.equals(artifactColor)){
                if((distance > 80) || ((hsvValues[0] < 10) && (hsvValues[1] < 0.1))) {
                    artifactColor = NO_ARTIFACT;
                }
            }
        } catch(Exception e) {
            //do nothing
        }

        return artifactColor;
    }

    public DecodeConstants.ArtifactColor getLaunchSlotArtifactColor() {
        return detectArtifactColor(0);
    }

    public DecodeConstants.ArtifactColor getPickupSlotArtifactColor() {
        return detectArtifactColor(2);
    }

    public DecodeConstants.ArtifactColor getMiddleSlotArtifactColor() {
        return detectArtifactColor(1);
    }

    public DecodeConstants.ArtifactColor getSlotArtifactColor(int slotidx) {
        return detectArtifactColor(slotidx);
    }

    public void pushArtifactForLaunch() {
        if (!pushServoAvailable) return;

        timeSinceLastPush.reset();

        setPushServoToExtendPos();
        setPushServoToRetractPos();
    }
    public void setPushServoToExtendPos(){
        if(!pushServoAvailable) return;

        pushServo.setPosition(pushServoExtendPos);

        ElapsedTime waitClock = new ElapsedTime();
        while (waitClock.time() < 0.45) {}
    }
    public void setPushServoToRetractPos(){
        if(!pushServoAvailable) return;

        pushServo.setPosition(pushServoRetractPos);

        ElapsedTime waitClock = new ElapsedTime();
        while (waitClock.time() < 0.35) {}
    }

    public double getPushServoPosition() {
        return (pushServoAvailable) ? pushServo.getPosition() : -1;
    }
    public void setRotateServoPosition(double targetPosition) {
        if(!rotateServoAvailable) return;

        if(rotateServo.getPosition() == targetPosition) return;

        rotateServo.setPosition(targetPosition);
        timeSinceLastRotate.reset();
    }

    public double getRotatePosition() {
        return (rotateServo == null) ? -1 : rotateServo.getPosition();
    }

    public String getContainerDisplayInfo() {
        String displayLines = "";
        int displayPushSlot = currentPushSlotPositionIndex +1;
        int displayPickupSlot = currentPickupSlotPositionIndex +1;

        if(rotateServoAvailable) {
            displayLines += "\n" + "Rotate servo position = " + String.format("%6.3f", getRotatePosition());
        } else {
            displayLines += "\n" + "Servo to rotate container not available!";
        }

        if(pushServoAvailable) {
            displayLines += "\n" + "Push servo position = " + String.format("%6.3f", getPushServoPosition());
        } else {
            displayLines += "\n" + "Servo to push artifact not available!";
        }

        displayLines += "\n" + "Launch slot " + " : "+ getLaunchSlotArtifactColor().toString() + " Distance:" + hsvDistances.get(0);
        displayLines += "\n" + "Middle slot " + " : "+ getMiddleSlotArtifactColor().toString() + " Distance:" + hsvDistances.get(1);
        displayLines += "\n" + "Pickup slot " + " : "+ getPickupSlotArtifactColor().toString() + " Distance:" + hsvDistances.get(2);

        return displayLines;
    }

    public void wrapupAuto(){
    }
}