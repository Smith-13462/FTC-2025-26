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

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.AUTO_OP_MODE;

public class ArtifactContainer {
    private ArrayList<DecodeConstants.ArtifactColor> containerSlots;
    private ArrayList<Double> rotateServoPositions;
    public Servo pushServo = null;
    public Servo rotateServo = null;
    static final double PUSH_SERVO_ERROR_TOLERANCE = 0.001;
    static final double ROTATE_SERVO_ERROR_TOLERANCE = 0.001;
    int currentSlot = 0;
    ElapsedTime timeSinceLastRotate = new ElapsedTime();
    ElapsedTime timeSinceLastPush = new ElapsedTime();
    private boolean pushServoAvailable = true,
            rotateServoAvailable= true;
    ArrayList<NormalizedColorSensor> colorSensors;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ArrayList<Boolean> colorSensorsAvailable;

    public double
            pushServoExtendPos = 1,
            pushServoRetractPos = 0,
            rotateServoInitialPos = 0.5,
            rotatePerSlot = 0.4,
            timeNeededtoLaunch = 0.5,
            timeNeededToRotate = 0.5;

    ArtifactContainer(HardwareMap hardwareMap, DecodeConstants.OP_MODE op_mode, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initColorSensors();
        //if(AUTO_OP_MODE.equals(op_mode)) {
        //    initializeContainerWithPresetValues();
        //}
        initializeContainer();

        try {
            pushServo = hardwareMap.get(Servo.class, "pushservo");
            //pushServo.setDirection(Servo.Direction.REVERSE);
            pushServo.setPosition(0);
            pushServoAvailable = true;
        } catch (Exception e) {
            pushServoAvailable = false;
            telemetry.addData("Init problem with ", "push servo");
//            telemetry.update();
        }

        try {
            rotateServo = hardwareMap.get(Servo.class, "rotateservo");
            rotateServo.setPosition(rotateServoInitialPos);
            rotateServoAvailable = true;
        } catch (Exception e) {
            rotateServoAvailable = false;
            telemetry.addData("Init problem with ", "rotate servo");
//            telemetry.update();
        }

        timeSinceLastRotate.reset();
        timeSinceLastPush.reset();
    }
    public void initializeContainerWithPresetValues() {
        containerSlots = new ArrayList<>();

        containerSlots.add(0, DecodeConstants.ArtifactColor.PURPLE_ARTIFACT);
        containerSlots.add(1, DecodeConstants.ArtifactColor.GREEN_ARTIFACT);
        containerSlots.add(2, DecodeConstants.ArtifactColor.PURPLE_ARTIFACT);
    }
    public void initializeContainer() {
        containerSlots = new ArrayList<>();

        for(int slotIdx = 0; slotIdx < 3; slotIdx++){
            containerSlots.add(slotIdx, detectArtifactColor(slotIdx));
        }

        rotateServoPositions = new ArrayList<>();
        rotateServoPositions.add(0, rotateServoInitialPos);
        rotateServoPositions.add(1, rotateServoInitialPos + rotatePerSlot);
        rotateServoPositions.add(2, rotateServoInitialPos - rotatePerSlot);
    }
    private void initColorSensors() {
        float gain = 2;
        colorSensors = new ArrayList<>();
        colorSensorsAvailable = new ArrayList<>();

        for(Integer colorSensorIdx=0; colorSensorIdx<3; colorSensorIdx++) {
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
            } catch (Exception e) {
                colorSensorsAvailable.add(colorSensorIdx, Boolean.FALSE);
            }
        }
    }

    public void setContainerToEmpty() {
        containerSlots.set(0, null);
        containerSlots.set(1, null);
        containerSlots.set(2, null);
    }

    public void updateContainerInfo() {
        for (int slotIdx = 0; slotIdx < 3; slotIdx++) {
            containerSlots.set(slotIdx, detectArtifactColor(slotIdx));
        }
    }

    public void rotateContainer(boolean clockwise){
        if(!rotateServoAvailable) return;

        if(timeSinceLastRotate.time() < timeNeededToRotate) return;

        double direction = 1;
        if(clockwise) {
            direction = 1;
        } else {
            direction = -1;
        }

        if(currentSlot > 2) {
            currentSlot = 0;
        } else if(currentSlot < 0){
            currentSlot = 2;
        }

        double targetPosition;
        targetPosition= rotateServoPositions.get(currentSlot);

        setRotateServoPosition(targetPosition);
        timeSinceLastRotate.reset();
    }

    public void rotateContainerToSpecificArtifact(DecodeConstants.ArtifactColor desiredArtifactColor) {
        if (!rotateServoAvailable) return;

        if (desiredArtifactColor == null) return;

        if (timeSinceLastRotate.time() < timeNeededToRotate) return;
        int targetArtifactSlot = -1;

        for (int slotIdx = 0; slotIdx < 3; slotIdx++) {
            DecodeConstants.ArtifactColor slotArtifactColor = detectArtifactColor(slotIdx);

            if (desiredArtifactColor.equals(slotArtifactColor)) {
                if ((targetArtifactSlot == -1) || (slotIdx == currentSlot)) {
                    targetArtifactSlot = slotIdx;
                } else if (Math.abs(slotIdx - currentSlot) < Math.abs(slotIdx - targetArtifactSlot)) {
                    targetArtifactSlot = slotIdx;
                }
            }
        }

        if(targetArtifactSlot != -1) {
            double targetPosition;
            targetPosition= rotateServoPositions.get(currentSlot);
            setRotateServoPosition(targetPosition);
        }
    }

    public void adjustRotationPerSlot(double adjustment){
        rotatePerSlot += adjustment;
    }

    public void pushArtifactForLaunch(int slotIdx) {
        if (!pushServoAvailable) return;

        ElapsedTime waitClock = new ElapsedTime();
        setPushServoToExtendPos();
        waitClock.reset();
        while (waitClock.time() < 1) {}
        setPushServoToRetractPos();
        waitClock.reset();
        while (waitClock.time() < 2) {}
        timeSinceLastPush.reset();
    }
    public void setPushServoToExtendPos(){
        if(!pushServoAvailable) return;

        pushServo.setPosition(pushServoExtendPos);
      //  pushServo.setDirection(Servo.Direction.FORWARD);

    }
    public void setPushServoToRetractPos(){
        if(!pushServoAvailable) return;

        pushServo.setPosition(pushServoRetractPos);
        //pushServo.setDirection(Servo.Direction.REVERSE);
    }

    public double getPushServoPosition() {
        return (pushServoAvailable) ? pushServo.getPosition() : -1;
    }
    public void setRotateServoPosition(double targetPosition) {
        if(!rotateServoAvailable) return;

        rotateServo.setPosition(targetPosition);
        timeSinceLastRotate.reset();
    }

    public double getRotatePosition() {
        return (rotateServo == null) ? -1 : rotateServo.getPosition();
    }

    public DecodeConstants.ArtifactColor detectArtifactColor(int slotIdx) {
        DecodeConstants.ArtifactColor artifactColor = DecodeConstants.ArtifactColor.UNKNOWN_ARTIFACT;

        if(colorSensorsAvailable.get(slotIdx) == Boolean.FALSE){
            return artifactColor;
        }

        try {
            final float[] hsvValues = new float[3];
            NormalizedRGBA colors = colorSensors.get(slotIdx).getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            double distance = 0;
            if (colorSensors.get(slotIdx) instanceof DistanceSensor) {
                distance = ((DistanceSensor) colorSensors.get(slotIdx)).getDistance(DistanceUnit.CM);
            }

            if ((hsvValues[0] >= 145) && (hsvValues[0] <= 175) && (hsvValues[1] >= 0.7)) {
                artifactColor = DecodeConstants.ArtifactColor.GREEN_ARTIFACT;
            } else if ((hsvValues[0] >= 210) && (hsvValues[1] >= 0.4)) {
                artifactColor = DecodeConstants.ArtifactColor.PURPLE_ARTIFACT;
            }
        } catch(Exception e) {
            //do nothing
        }

        return artifactColor;
    }

    public DecodeConstants.ArtifactColor getCurrentSlotArtifactColor(){
        return detectArtifactColor(currentSlot);
    }

    public int getCurrentSlotNumber() {
        return  currentSlot;
    }

    public String getContainerDisplayInfo() {
        String displayLines = "";

        if(rotateServoAvailable) {
            displayLines += "\n" + "Rotate Servo position = " + String.format("%6.3f", getRotatePosition());
            displayLines += "\n" + "Current Slot = " + currentSlot+1;
        } else {
            displayLines += "\n" + "Servo to rotate container not available!";
        }

        if(pushServoAvailable) {
            displayLines += "\n" + "Push Servo position = " + String.format("%6.3f", getPushServoPosition());
        } else {
            displayLines += "\n" + "Servo to push artifact not available!";
        }

       displayLines += "\n" + "Container slots: "  + detectArtifactColor(0) + ", "
               + detectArtifactColor(1) + ", " + detectArtifactColor(2);

        return displayLines;
    }
}