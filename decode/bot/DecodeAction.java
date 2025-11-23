package org.firstinspires.ftc.teamcode.decode.bot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Action.AUTOOP_LAUNCH;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Action.AUTO_LAUNCH;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Action.JUST_LAUNCH;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Action.TURN_TO_LAUNCH;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.GREEN_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.NO_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.PURPLE_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.UNKNOWN_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.BLUE_PICKUP_PPG_SPIKE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.FieldPosition.RED_PICKUP_PPG_SPIKE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Motif.GPP_MOTIF;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Motif.PGP_MOTIF;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Motif.PPG_MOTIF;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Motif.UNKNOWN_MOTIF;

public class DecodeAction {
    private DecodeBot owlsBotDecode;
    private Telemetry telemetry;
    private static DecodeConstants.Action currentAction = null;
    private ElapsedTime winddownClock = new ElapsedTime();
    private DecodeConstants.TeamAllianceColor teamAllianceColor;
    private double timeNeededToCompleteAction = 0;

    public DecodeAction(DecodeBot owlsBotDecode, Telemetry telemetry,
                        DecodeConstants.TeamAllianceColor teamAllianceColor) {
        this.telemetry = telemetry;
        this.owlsBotDecode = owlsBotDecode;
        this.teamAllianceColor = teamAllianceColor;

        winddownClock.reset();
        timeNeededToCompleteAction = 0;
    }

    public Runnable parallelAlignPickupToEmpty = new Runnable() {
        @Override
        public void run() {

            boolean rotationDone = false;
            if ((owlsBotDecode.getContainer().getNumberOfEmptySlots() < 3)) {
                rotationDone = alignEmptySlotToPickup(false);
            }
        }
    };

    public void autoLaunchPreloaded(DecodeConstants.FieldPosition launchPosition, double timeAvailable, boolean launchSpeedAlreadySet,
                                    boolean firstRotateAlreadyDone, double maxWaitTimeForSpeedAdjustment, boolean retryLaunch) {
        currentAction = AUTOOP_LAUNCH;

        owlsBotDecode.getMotion().turnToLaunch();

        DecodeConstants.Motif launchMotif = owlsBotDecode.getVision().getMotif();

        int firstRotateDirection = 0, secondRotateDirection = 0, thirdRotateDirection = 0;
        if (PGP_MOTIF.equals(launchMotif)) {
            firstRotateDirection = 0;
            secondRotateDirection = 1;
            thirdRotateDirection = 1;
        }
        if (PPG_MOTIF.equals(launchMotif)) {
            firstRotateDirection = 0;
            secondRotateDirection = -1;
            thirdRotateDirection = -1;
        }
        if (GPP_MOTIF.equals(launchMotif)) {
            firstRotateDirection = 1;
            secondRotateDirection = 1;
            thirdRotateDirection = 1;
        }

        for (int motifIdx = 0; motifIdx < 3; motifIdx++) {

            boolean rotateStarted = false;
            double minWaitTime = 0;

            if (motifIdx == 0) {
                if(!firstRotateAlreadyDone){
                    if (firstRotateDirection == 1) {
                        owlsBotDecode.getContainer().rotateContainer(true, false);
                        rotateStarted = true;
                    } else if (firstRotateDirection == -1) {
                        owlsBotDecode.getContainer().rotateContainer(false, false);
                        rotateStarted = true;
                    }
                }
            } else if (motifIdx == 1) {
                if (secondRotateDirection == 1) {
                    owlsBotDecode.getContainer().rotateContainer(true, false);
                    rotateStarted = true;
                } else if (secondRotateDirection == -1) {
                    owlsBotDecode.getContainer().rotateContainer(false, false);
                    rotateStarted = true;
                }
            } else {
                if (thirdRotateDirection == 1) {
                    owlsBotDecode.getContainer().rotateContainer(true, false);
                    rotateStarted = true;
                } else if (thirdRotateDirection == -1) {
                    owlsBotDecode.getContainer().rotateContainer(false, false);
                    rotateStarted = true;
                }
            }

            if (rotateStarted) {
                minWaitTime = 1;
            } else {
                minWaitTime = 0.75;
            }

            autoLaunchArtifact(null, minWaitTime, null, launchSpeedAlreadySet,
                    true, false, maxWaitTimeForSpeedAdjustment
            ,retryLaunch);
        }

        if(owlsBotDecode.getContainer().getNumberOfEmptySlots() >0){
            autoLaunchArtifact(UNKNOWN_ARTIFACT, 0, null, launchSpeedAlreadySet,
                    true, false, maxWaitTimeForSpeedAdjustment, retryLaunch);
        }

        timeNeededToCompleteAction = 0;
        winddownClock.reset();
    }

    public void autoLaunchArtifact(DecodeConstants.ArtifactColor launchArtifactColor, double minWaitTimeBeforeLaunch,
                                   DecodeConstants.FieldPosition launchPosition, boolean launchSpeedAlreadySet,
                                   boolean turnAlreadyDone,
                                   boolean holdFinalStep, double maxWaitTimeForSpeedAdjustment,
                                   boolean retry) {
        currentAction = AUTO_LAUNCH;
        ElapsedTime autoOpLaunchStart = new ElapsedTime();
        autoOpLaunchStart.reset();

        owlsBotDecode.getIntake().turnOffIntake();

        boolean anyVelocityAdjustmentDone = false, anySlotRotationDone = false;
        Pose currentBotPinPointPose = owlsBotDecode.getMotion().getUpdatedBotPose();
        ElapsedTime velocityTimer = new ElapsedTime();
        velocityTimer.reset();
        if(launchSpeedAlreadySet){
            //do nothing
        } else {
            if (launchPosition != null) {
                anyVelocityAdjustmentDone = owlsBotDecode.getLauncher().setPropulsionSpeed(launchPosition);
            } else if (currentBotPinPointPose != null) {
                anyVelocityAdjustmentDone = owlsBotDecode.getLauncher().setPropulsionSpeed(currentBotPinPointPose);
            }
        }

        ElapsedTime rotateTimer = new ElapsedTime();
        rotateTimer.reset();
        if(PURPLE_ARTIFACT.equals(launchArtifactColor) || GREEN_ARTIFACT.equals(launchArtifactColor)) {
            anySlotRotationDone = owlsBotDecode.getContainer().alignToLaunchSpecificArtifact(launchArtifactColor, false);
        } else {
            if(NO_ARTIFACT.equals(owlsBotDecode.getContainer().getLaunchSlotArtifactColor())) {
                if (owlsBotDecode.getContainer().isSpecificArtifactInContainer(PURPLE_ARTIFACT)){
                    anySlotRotationDone = owlsBotDecode.getContainer().alignToLaunchSpecificArtifact(PURPLE_ARTIFACT, false);
                } else if(owlsBotDecode.getContainer().isSpecificArtifactInContainer(GREEN_ARTIFACT)){
                    anySlotRotationDone = owlsBotDecode.getContainer().alignToLaunchSpecificArtifact(GREEN_ARTIFACT, false);
                } else if(owlsBotDecode.getContainer().isSpecificArtifactInContainer(UNKNOWN_ARTIFACT)){
                    anySlotRotationDone = owlsBotDecode.getContainer().alignToLaunchSpecificArtifact(UNKNOWN_ARTIFACT, false);
                }
            } else if(UNKNOWN_ARTIFACT.equals(owlsBotDecode.getContainer().getLaunchSlotArtifactColor())) {
                if (owlsBotDecode.getContainer().isSpecificArtifactInContainer(PURPLE_ARTIFACT)){
                    anySlotRotationDone = owlsBotDecode.getContainer().alignToLaunchSpecificArtifact(PURPLE_ARTIFACT, false);
                } else if(owlsBotDecode.getContainer().isSpecificArtifactInContainer(GREEN_ARTIFACT)){
                    anySlotRotationDone = owlsBotDecode.getContainer().alignToLaunchSpecificArtifact(GREEN_ARTIFACT, false);
                }
            }
        }

        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();
        boolean anyTurnDone = false;
        if(turnAlreadyDone){
            //do nothing
        } else {
            anyTurnDone = owlsBotDecode.getMotion().turnToLaunch();
        }

        while(((!owlsBotDecode.getLauncher().isAtDesiredSpeedForLaunch()) && (velocityTimer.time() < maxWaitTimeForSpeedAdjustment)) ||
                (anySlotRotationDone && (rotateTimer.time() < 1)) ||
                (anyTurnDone && (turnTimer.time() < 0.5)) ||
                (autoOpLaunchStart.time() < minWaitTimeBeforeLaunch)){
            //wait
        }

        if(!holdFinalStep) {
            owlsBotDecode.getContainer().pushArtifactForLaunch();
            if(retry){
                retryLaunchArtifact();
            }
            timeNeededToCompleteAction = 0;
        } else {
            timeNeededToCompleteAction = 0;
        }
        winddownClock.reset();
    }

    public void rotateToPrepareForNextLaunch() {
        if (owlsBotDecode.getContainer().isSpecificArtifactInContainer(PURPLE_ARTIFACT)){
            owlsBotDecode.getContainer().alignToLaunchSpecificArtifact(PURPLE_ARTIFACT, false);
        } else if(owlsBotDecode.getContainer().isSpecificArtifactInContainer(GREEN_ARTIFACT)){
            owlsBotDecode.getContainer().alignToLaunchSpecificArtifact(GREEN_ARTIFACT, false);
        } else if(owlsBotDecode.getContainer().isSpecificArtifactInContainer(UNKNOWN_ARTIFACT)){
            owlsBotDecode.getContainer().alignToLaunchSpecificArtifact(UNKNOWN_ARTIFACT, false);
        }
    }

    public void justLaunch() {
        if(isDelayNeededToCompleteLastAction()) return;
        currentAction = JUST_LAUNCH;

        owlsBotDecode.getIntake().turnOffIntake();
        ElapsedTime velocityTimer = new ElapsedTime();
        velocityTimer.reset();
        while(!owlsBotDecode.getLauncher().isAtDesiredSpeedForLaunch() && (velocityTimer.time() < 2)){
            //wait
        }
        owlsBotDecode.getContainer().pushArtifactForLaunch();

        timeNeededToCompleteAction = 0;
        winddownClock.reset();
    }

    public void retryLaunchArtifact() {
        if(owlsBotDecode.getContainer().isLaunchSlotFilledWithArtifact()){
            owlsBotDecode.getContainer().pushArtifactForLaunch();
        }
    }

    public void turnToLaunch(){
        if(isDelayNeededToCompleteLastAction()) return;
        currentAction = TURN_TO_LAUNCH;

        owlsBotDecode.getMotion().turnToLaunch();

        timeNeededToCompleteAction = 0.25;
        winddownClock.reset();
    }

    public void autoOpPickupAll(DecodeConstants.FieldPosition pickupPosition) {

        ElapsedTime timeSincePickupStart = new ElapsedTime();
        timeSincePickupStart.reset();

        owlsBotDecode.getIntake().turnOnIntake();

        double maxMovement = 26;

        owlsBotDecode.getMotion().moveStraightToPickupAll(teamAllianceColor, pickupPosition,maxMovement, parallelAlignPickupToEmpty);
    }

    public boolean alignEmptySlotToPickup(boolean waitUntilRotationIsComplete)  {
        boolean pickupSlotEmpty = false, launchSlotEmpty = false, middleSlotEmpty = false;
        boolean rotationDone = false;

        DecodeConstants.ArtifactColor launchSlotArtifactColor = owlsBotDecode.getContainer().getLaunchSlotArtifactColor();
        DecodeConstants.ArtifactColor middleSlotArtifactColor = owlsBotDecode.getContainer().getMiddleSlotArtifactColor();
        DecodeConstants.ArtifactColor pickupSlotArtifactColor = owlsBotDecode.getContainer().getPickupSlotArtifactColor();

        if(NO_ARTIFACT.equals(pickupSlotArtifactColor)) {
            pickupSlotEmpty = true;
        }

        if(NO_ARTIFACT.equals(middleSlotArtifactColor)) {
            middleSlotEmpty = true;
        }

        if(NO_ARTIFACT.equals(launchSlotArtifactColor)) {
            launchSlotEmpty = true;
        }

        if(pickupSlotEmpty) {
        } else if(launchSlotEmpty) {
            owlsBotDecode.getContainer().rotateContainer(true, waitUntilRotationIsComplete);
            rotationDone = true;
        } else if(middleSlotEmpty) {
            owlsBotDecode.getContainer().rotateContainer(false, waitUntilRotationIsComplete);
            rotationDone = true;
        }

        return rotationDone;
    }

    public void autoLaunchAll(DecodeConstants.FieldPosition launchPosition, double timeAvailable,
                              boolean launchSpeedAlreadySet, double maxWaitTimeForSpeedAdjustment
                                ,boolean retryLaunch) {
        currentAction = AUTOOP_LAUNCH;
        ElapsedTime maxWaitTimeForPreviousAction = new ElapsedTime();
        maxWaitTimeForPreviousAction.reset();
        ElapsedTime timeSinceAutoLaunchStart = new ElapsedTime();
        timeSinceAutoLaunchStart.reset();

        owlsBotDecode.getMotion().turnToLaunch();

        DecodeConstants.Motif launchMotif = owlsBotDecode.getVision().getMotif();
        if((launchMotif == null) || (UNKNOWN_MOTIF.equals(launchMotif))) {
            launchMotif =  PGP_MOTIF;
        }

        ArrayList<Integer> alreadyTriedSlots = new ArrayList();
        DecodeConstants.ArtifactColor desiredArtifactColor = null, otherArtifactColor = null;
        for(int motifIdx = 0; motifIdx < 3; motifIdx++) {
            if(timeSinceAutoLaunchStart.time() > (timeAvailable - (maxWaitTimeForSpeedAdjustment))) { continue;}

            if (launchMotif.toString().charAt(motifIdx) == 'P') {
                desiredArtifactColor = PURPLE_ARTIFACT;
                otherArtifactColor = GREEN_ARTIFACT;
            } else {
                desiredArtifactColor = GREEN_ARTIFACT;
                otherArtifactColor = PURPLE_ARTIFACT;
            }

            Integer untriedDesiredArtifactColorSlot = null, untriedOtherArtifactColorSlot = null, untriedUnknownColorSlot = null;
            for(Integer slotIdx = 0; slotIdx < 3; slotIdx++) {
                if ((alreadyTriedSlots.size() !=0) && alreadyTriedSlots.contains(slotIdx.intValue())){
                    continue;
                }

                if (desiredArtifactColor.equals(owlsBotDecode.getContainer().getSlotArtifactColor(slotIdx.intValue()))) {
                    if (slotIdx.intValue() == owlsBotDecode.getContainer().getLaunchSlotPositionIndex()) {
                        untriedDesiredArtifactColorSlot = slotIdx;
                    } else {
                        if (untriedDesiredArtifactColorSlot == null) {
                            untriedDesiredArtifactColorSlot = slotIdx;
                        }
                    }
                } else if (otherArtifactColor.equals(owlsBotDecode.getContainer().getSlotArtifactColor(slotIdx.intValue()))) {
                    if (slotIdx.intValue() == owlsBotDecode.getContainer().getLaunchSlotPositionIndex()) {
                        untriedOtherArtifactColorSlot = slotIdx;
                    } else {
                        if (untriedOtherArtifactColorSlot == null) {
                            untriedOtherArtifactColorSlot = slotIdx;
                        }
                    }
                } else {
                    if (slotIdx.intValue() == owlsBotDecode.getContainer().getLaunchSlotPositionIndex()) {
                        untriedUnknownColorSlot = slotIdx;
                    } else {
                        if (untriedUnknownColorSlot == null) {
                            untriedUnknownColorSlot = slotIdx;
                        }
                    }
                }
            }

            boolean rotateStarted = false;
            double minWaitTime = 0;

            if(untriedDesiredArtifactColorSlot != null) {
                rotateStarted = owlsBotDecode.getContainer().rotateToAlignSlotForLaunch(untriedDesiredArtifactColorSlot.intValue(), false);
                if(rotateStarted) {
                    minWaitTime = 1;
                } else {
                    minWaitTime = 0.75;
                }
                autoLaunchArtifact(null, minWaitTime, null, launchSpeedAlreadySet,
                        true, false, maxWaitTimeForSpeedAdjustment
                ,retryLaunch);
                alreadyTriedSlots.add(untriedDesiredArtifactColorSlot);
            } else if(untriedUnknownColorSlot != null){
                rotateStarted = owlsBotDecode.getContainer().rotateToAlignSlotForLaunch(untriedUnknownColorSlot.intValue(), false);
                if(rotateStarted) {
                    minWaitTime = 1;
                } else {
                    minWaitTime = 0.75;
                }
                autoLaunchArtifact(null, minWaitTime, null, launchSpeedAlreadySet,
                        true,false, maxWaitTimeForSpeedAdjustment,retryLaunch);
                alreadyTriedSlots.add(untriedUnknownColorSlot);
            }  else if(untriedOtherArtifactColorSlot != null){
                rotateStarted = owlsBotDecode.getContainer().rotateToAlignSlotForLaunch(untriedOtherArtifactColorSlot.intValue(), false);
                if(rotateStarted) {
                    minWaitTime = 1;
                } else {
                    minWaitTime = 0.75;
                }
                autoLaunchArtifact(null, minWaitTime, null, launchSpeedAlreadySet,
                        true, false, maxWaitTimeForSpeedAdjustment, retryLaunch);
                alreadyTriedSlots.add(untriedOtherArtifactColorSlot);
            }
        }

        if((owlsBotDecode.getContainer().getNumberOfEmptySlots() < 3) && retryLaunch &&
            (timeSinceAutoLaunchStart.time() < (timeAvailable - 1))) {
            autoLaunchArtifact(UNKNOWN_ARTIFACT, 0, null, launchSpeedAlreadySet,
                    true, false, 0, false);
        }

        ElapsedTime endClock = new ElapsedTime();
        endClock.reset();
        //while(endClock.time() < 0.75) {}
        timeNeededToCompleteAction = 0;
        winddownClock.reset();
    }

    public boolean isDelayNeededToCompleteLastAction() {
        if(winddownClock.time() < (timeNeededToCompleteAction)) {
            return true;
        } else {
            currentAction = null;
            timeNeededToCompleteAction = 0;
            return false;
        }
    }

}