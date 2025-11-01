package org.firstinspires.ftc.teamcode.decode.bot;

import android.sax.StartElementListener;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.GREEN_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.ArtifactColor.PURPLE_ARTIFACT;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Motif.PGP_MOTIF;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.Motif.UNKNOWN_MOTIF;

public class DecodeAction {
    private DecodeBot owlsBotDecode;
    private Telemetry telemetry;
    public DecodeAction(DecodeBot owlsBotDecode, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.owlsBotDecode = owlsBotDecode;
    }

    public Runnable rotateToSpecificColor = new Runnable() {
        @Override
        public void run() {
            // rotate to specific color
        }
    };

    public void autoLaunch(DecodeConstants.Motif launchMotif) {
        if((launchMotif == null) || (UNKNOWN_MOTIF.equals(launchMotif))) {
            launchMotif =  PGP_MOTIF;
        }

        ArrayList<Integer> alreadyTriedSlots = new ArrayList();
        DecodeConstants.ArtifactColor desiredArtifactColor = null, otherArtifactColor = null;
        for(int motifIdx = 0; motifIdx < 3; motifIdx++) {
            if (launchMotif.toString().charAt(motifIdx) == 'P') {
                desiredArtifactColor = PURPLE_ARTIFACT;
                otherArtifactColor = GREEN_ARTIFACT;
            } else {
                desiredArtifactColor = GREEN_ARTIFACT;
                otherArtifactColor = PURPLE_ARTIFACT;
            }

            owlsBotDecode.getContainer().updateContainerInfo();

            Integer untriedDesiredArtifactColorSlot = null, untriedOtherArtifactColorSlot = null, untriedUnknownColorSlot = null;
            for(int slotIdx = 0; slotIdx < 3; slotIdx++) {
                if (alreadyTriedSlots.contains(slotIdx)) continue;

                if (desiredArtifactColor.equals(owlsBotDecode.getContainer().getCurrentSlotArtifactColor())) {
                    if (slotIdx == owlsBotDecode.getContainer().getCurrentSlotNumber()) {
                        untriedDesiredArtifactColorSlot = slotIdx;
                    } else {
                        if (untriedDesiredArtifactColorSlot != null) {
                            untriedDesiredArtifactColorSlot = slotIdx;
                        }
                    }
                } else if (otherArtifactColor.equals(owlsBotDecode.getContainer().getCurrentSlotArtifactColor())) {
                    if (slotIdx == owlsBotDecode.getContainer().getCurrentSlotNumber()) {
                        untriedOtherArtifactColorSlot = slotIdx;
                    } else {
                        if (untriedOtherArtifactColorSlot != null) {
                            untriedOtherArtifactColorSlot = slotIdx;
                        }
                    }
                } else if (slotIdx == owlsBotDecode.getContainer().getCurrentSlotNumber()) {
                    untriedUnknownColorSlot = slotIdx;
                } else if (untriedUnknownColorSlot != null) {
                    untriedUnknownColorSlot = slotIdx;
                }
            }

            if(untriedDesiredArtifactColorSlot != null) {
                //rotate to slot
                owlsBotDecode.getContainer().pushArtifactForLaunch(untriedDesiredArtifactColorSlot);
                alreadyTriedSlots.add(untriedDesiredArtifactColorSlot);
            } else if(untriedUnknownColorSlot != null){
                //rotate to slot
                owlsBotDecode.getContainer().pushArtifactForLaunch(untriedUnknownColorSlot);
                alreadyTriedSlots.add(untriedUnknownColorSlot);
            }  else if(untriedOtherArtifactColorSlot != null){
                //rotate to slot
                owlsBotDecode.getContainer().pushArtifactForLaunch(untriedOtherArtifactColorSlot);
                alreadyTriedSlots.add(untriedOtherArtifactColorSlot);
            }
        }
    }
}