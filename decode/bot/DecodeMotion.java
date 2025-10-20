package org.firstinspires.ftc.teamcode.decode.bot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;
import org.firstinspires.ftc.teamcode.decode.common.DecodeUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.RED_ALLIANCE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConfig.*;

public class DecodeMotion {
    HardwareMap hardwareMap;
    GoBildaPinpointDriver pinpoint;
    boolean pinpointAvailable = false, pedropathDrive = false;
    private Follower pedroPathFollower;
    private Path pedroPathTravelCurve;
    Telemetry telemetry = null;
    private Pose startPose;
    private DecodeBot owlsBotDecode;
    private static Pose lastKnownBotPose = null;
    private boolean botPositionOnHold = false;
    private double acceptableDelta = 1;

    DecodeMotion(HardwareMap hardwareMap, DecodeBot owlsBotDecode, boolean automatedDrive, boolean recalibratePinpoint,
                 Pose pStartPose, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.pedropathDrive = automatedDrive;
        this.telemetry = telemetry;
        this.owlsBotDecode = owlsBotDecode;

        if(pStartPose == null) {
            if(lastKnownBotPose == null) {
                this.startPose = new Pose(0,0, 0);
            } else {
                this.startPose = new Pose(lastKnownBotPose.getX(), lastKnownBotPose.getY(), lastKnownBotPose.getHeading());
            }
        } else {
            this.startPose = new Pose(pStartPose.getX(), pStartPose.getY(), pStartPose.getHeading());
        }

        //pinpoint
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpointAvailable = true;
            configurePinpoint(recalibratePinpoint);
            //pinpoint.setPosition(startPose);
            pinpoint.update();
        } catch (Exception e){
            pinpointAvailable = false;
            this.pedropathDrive = false;
        }

        if(pedropathDrive) {
            pedroPathFollower = Constants.createFollower(hardwareMap);
            pedroPathFollower.setStartingPose(new Pose(this.startPose.getX(), this.startPose.getY(), this.startPose.getHeading()));
            pedroPathFollower.update();
            lastKnownBotPose = pedroPathFollower.getPose();
        }
    }

    public void configurePinpoint(boolean recalibrate){
        pinpoint.setOffsets(2.83, -6.61, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        if(recalibrate) {
            pinpoint.resetPosAndIMU();
        }
        ElapsedTime hold = new ElapsedTime();
        hold.reset();
        while(hold.time() < 2) {
            telemetry.addLine("pinpoint recalibration in progress....");
            telemetry.update();
        }
    }

    public Pose2D getPinPointPose(){
        if(pinpointAvailable) {
            pinpoint.update();
            return pinpoint.getPosition();
        } else {
            return null;
        }
    }
    public void turnToLaunch(TeamAllianceColor teamAllianceColor){
        moveLinearOrTurnToLaunch(teamAllianceColor, 0, 0, true);
    }

    public void turnOffPositionHold(TeamAllianceColor teamAllianceColor) {
        if(!this.botPositionOnHold) return;

        pedroPathFollower.breakFollowing();
        pedroPathFollower.update();
        Pose currentBotPosePedro = new Pose(pedroPathFollower.getPose().getX(), pedroPathFollower.getPose().getY(),
                pedroPathFollower.getPose().getHeading());


        PathChain turnOffHoldPathChain =  pedroPathFollower.pathBuilder()
                .addPath(new BezierLine(currentBotPosePedro,
                        new Pose(currentBotPosePedro.getX() + 0.01,currentBotPosePedro.getY() + 0.01,
                                currentBotPosePedro.getHeading())
                ))
                .setLinearHeadingInterpolation(currentBotPosePedro.getHeading(),
                        currentBotPosePedro.getHeading(),
                        0.8)
                .build();

        pedroPathFollower.followPath(turnOffHoldPathChain, false);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while ((pedroPathFollower.isBusy() || pedroPathFollower.getDistanceRemaining() > 0 || pedroPathFollower.isTurning())
                && (timer.time() < 0.1)) {
            pedroPathFollower.update();
            telemetry.addLine("Turning off bot position hold...");
            telemetry.update();
        }

        pedroPathFollower.breakFollowing();
        lastKnownBotPose = pedroPathFollower.getPose();

        this.botPositionOnHold = false;
    }
    public void moveLinearOrTurnToLaunch(TeamAllianceColor teamAllianceColor, double left, double forward, boolean hold){
        double turnAngle = 0;
        boolean leftTurn = false;

        //pedroPathFollower.breakFollowing();
        //pedroPathFollower.update();
        Pose currentBotPosePedro = new Pose(pedroPathFollower.getPose().getX(), pedroPathFollower.getPose().getY(),
                pedroPathFollower.getPose().getHeading());
        turnAngle = DecodeUtil.getLaunchHeadingAdjustment(teamAllianceColor,
                currentBotPosePedro, left, forward);

        if((left == 0) && (forward == 0)) {
            if(Math.abs(turnAngle) > 1) {
                leftTurn = (turnAngle > 0) ? true : false;

                pedroPathFollower.turnDegrees(Math.abs(turnAngle), leftTurn);

                ElapsedTime wait = new ElapsedTime();
                wait.reset();

            } else {
                telemetry.addLine("No turn needed. Already at an angle to launch...");
                telemetry.update();
            }
        } else {
            PathChain moveAndTurnPathChain =  pedroPathFollower.pathBuilder()
                    .addPath(new BezierLine(currentBotPosePedro,
                            new Pose(currentBotPosePedro.getX() + forward,currentBotPosePedro.getY() + left,
                                    Math.toRadians(Math.toDegrees(currentBotPosePedro.getHeading()) + turnAngle))
                    ))
                    .setLinearHeadingInterpolation(currentBotPosePedro.getHeading(),
                            Math.toRadians(Math.toDegrees(currentBotPosePedro.getHeading()) + turnAngle),
                            0.8)
                    // .addParametricCallback(0.5, testRunnable)
                    .build();

            pedroPathFollower.followPath(moveAndTurnPathChain, hold);
            pedroPathFollower.update();
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while ((pedroPathFollower.isBusy() || pedroPathFollower.getDistanceRemaining() > 0 || pedroPathFollower.isTurning())
        && (timer.time() < 1.5)) {
            pedroPathFollower.update();
            telemetry.addLine("Moving or turning " + (leftTurn ? "left " : "right ") + Math.abs(turnAngle) + " degrees.");
            telemetry.update();
        }

        //secondary correction
        timer.reset();
        while(timer.time() < 0.5) {
            telemetry.addLine("Moving or turning " + (leftTurn ? "left " : "right ") + Math.abs(turnAngle) + " degrees.");
            telemetry.update();
        }

       // pedroPathFollower.breakFollowing();
       // pedroPathFollower.update();
        this.botPositionOnHold = hold ? true : false;
        lastKnownBotPose = pedroPathFollower.getPose();
    }

    public void travelToLaunchOrPickup(ArrayList<Pose> travelPathControlPoints){

        if(travelPathControlPoints.size() > 2) {
            PathChain travelPathChain =  pedroPathFollower.pathBuilder()
                    .addPath(new BezierCurve(travelPathControlPoints))
                    .setLinearHeadingInterpolation(travelPathControlPoints.get(0).getHeading(),
                            travelPathControlPoints.get(travelPathControlPoints.size() - 1).getHeading(), 0.8)
                    .build();

            pedroPathFollower.followPath(travelPathChain);
            while(pedroPathFollower.isBusy() || pedroPathFollower.isTurning() || (pedroPathFollower.getDistanceRemaining() > 0)) {
                pedroPathFollower.update();
            };
        } else if (travelPathControlPoints.size() == 2) {
            PathChain travelPathChain =  pedroPathFollower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(travelPathControlPoints.get(0).getX(),
                                    travelPathControlPoints.get(0).getY(),
                                    travelPathControlPoints.get(0).getHeading()),
                            new Pose(travelPathControlPoints.get(1).getX(),
                                    travelPathControlPoints.get(1).getY(),
                                    travelPathControlPoints.get(1).getHeading())
                    ))
                    .setLinearHeadingInterpolation(travelPathControlPoints.get(0).getHeading(),
                            travelPathControlPoints.get(1).getHeading(), 0.8)
                    .build();

            pedroPathFollower.followPath(travelPathChain);
            while(pedroPathFollower.isBusy() || pedroPathFollower.isTurning() || (pedroPathFollower.getDistanceRemaining() > 0)) {
                pedroPathFollower.update();
            };
        }

        pedroPathFollower.update();
        lastKnownBotPose = pedroPathFollower.getPose();
    }

    public void travelToLaunchOrPickup(ArrayList<Pose> travelPathControlPoints, boolean retry) {
        travelToLaunchOrPickup(travelPathControlPoints);

        if(retry && (travelPathControlPoints.size() > 0) &&
            !isBotPoseWithinAcceptableDelta(travelPathControlPoints.get(travelPathControlPoints.size() - 1))) {
            ArrayList<Pose> retryControlPoints = new ArrayList<>();
            Pose currentPose = getUpdatedBotPose();
            retryControlPoints.set(0, currentPose);
            retryControlPoints.set(1, travelPathControlPoints.get(travelPathControlPoints.size() - 1));
            travelToLaunchOrPickup(retryControlPoints);
        }
    }

    public void travelToLaunchOrPickup(TravelPathShape pathShape, Pose toPose, boolean retry){

        ArrayList<Pose> travelPathControlPoints =  new ArrayList<>();

        Pose currentPose = getUpdatedBotPose();

        travelPathControlPoints.set(0, currentPose);

        if(pathShape.equals(TravelPathShape.CURVE)) {
            Pose intermediatePoint = new Pose((currentPose.getX() + toPose.getX()) / 2,
                    (currentPose.getY() + toPose.getY()) / 2,
                    toPose.getHeading());
            travelPathControlPoints.set(1, intermediatePoint);
            travelPathControlPoints.set(2, toPose);
        } else {
            travelPathControlPoints.set(1, toPose);
        }

        travelToLaunchOrPickup(travelPathControlPoints, retry);
    }

    public void travelToLaunchOrPickup(TravelPathShape pathShape, FieldPosition toPosition, boolean retry){

        ArrayList<Pose> travelPathControlPoints =  new ArrayList<>();

        Pose toPose = DecodeConfig.getFieldPositionPose(toPosition);

        if(toPose != null) {
            travelToLaunchOrPickup(pathShape, toPose, retry);
        }
    }

    public void travelToLaunchOrPickup(TravelPathShape pathShape, FieldPosition fromPosition, FieldPosition toPosition, boolean retry) {
        ArrayList<Pose> travelPathControlPoints =  DecodeConfig.getCurvePoints(fromPosition, toPosition);

        if((travelPathControlPoints == null) || (travelPathControlPoints.size() == 1)) {
            travelToLaunchOrPickup(pathShape, toPosition, retry);
            return;
        }

        if((pathShape.equals(TravelPathShape.CURVE)) && (travelPathControlPoints.size() == 2)) {
            Pose intermediatePoint = new Pose((travelPathControlPoints.get(0).getX() + travelPathControlPoints.get(1).getX()) / 2,
                    (travelPathControlPoints.get(0).getY() + travelPathControlPoints.get(1).getY()) / 2,
                    travelPathControlPoints.get(1).getHeading());
            travelPathControlPoints.set(2, travelPathControlPoints.get(1));
            travelPathControlPoints.set(1, intermediatePoint);
        }

        if(!isBotPoseWithinAcceptableDelta(travelPathControlPoints.get(0))) {
            travelPathControlPoints.set(0, new Pose(lastKnownBotPose.getX(), lastKnownBotPose.getY(), lastKnownBotPose.getHeading()));
        }

        travelToLaunchOrPickup(travelPathControlPoints, retry);
    }

    public boolean isBotPoseWithinAcceptableDelta(Pose targetPose) {
        boolean withinDelta = false;
        Pose currentPose = getUpdatedBotPose();

        if ((Math.abs(targetPose.getX() - currentPose.getX()) <= acceptableDelta) &&
                (Math.abs(targetPose.getY() - currentPose.getY()) <= acceptableDelta)) {
            withinDelta = true;
        }

        return withinDelta;
    }

    public void resetPose(Pose resetPose ) {
        pedroPathFollower.setStartingPose(new Pose(resetPose.getX(), resetPose.getY(),
                resetPose.getHeading()));
        pedroPathFollower.update();

        lastKnownBotPose = pedroPathFollower.getPose();
    }

    public Pose getUpdatedBotPose() {
        pedroPathFollower.update();
        lastKnownBotPose = pedroPathFollower.getPose();
        return lastKnownBotPose;
    }

    public Pose getLastKnownBotPose(){
        return lastKnownBotPose;
    }

    public String getBotPositionDisplayInfo(TeamAllianceColor teamAllianceColor, boolean displayPosition) {
        pedroPathFollower.update();
        Pose latestPedroPose = pedroPathFollower.getPose();
        String launchPositionName = getLaunchPositionName(latestPedroPose);
        String displayInfo = "";
        double turnAngle = DecodeUtil.getLaunchHeadingAdjustment(teamAllianceColor,
                latestPedroPose, 0, 0);
           try {
               if(displayPosition) {
                   displayInfo += "\n" + "Bot Start Position:"
                           + String.format("%6.1f", startPose.getX())
                           + "," + String.format("%6.1f", startPose.getY())
                           + "," + String.format("%6.1f", Math.toDegrees(startPose.getHeading()));
                   displayInfo += "\n" + "Bot Position:"
                           + String.format("%6.1f", latestPedroPose.getX())
                           + "," + String.format("%6.1f", latestPedroPose.getY())
                           + "," + String.format("%6.1f", Math.toDegrees(latestPedroPose.getHeading()));
               }
               displayInfo += "\nBot launch position: " + launchPositionName;
               displayInfo += "\nTurn needed to launch is" + String.format("%6.1f ", Math.abs(turnAngle)) + ((turnAngle > 0) ? "degrees left" : "degrees right");
            if (this.botPositionOnHold) {
                displayInfo +="\nPress DPad DOWN to resume steering...";
            }
            //telemetry.update();
        } catch (Exception ex) {
               displayInfo += "\n!!!Bot Position not available!!!";
            //telemetry.update();
        }

           return displayInfo;
    }

    public String getLaunchPositionName(Pose currentPose) {

        if(currentPose == null) return "None";

        if(currentPose.getX() <= 24)  {
            if((currentPose.getY() >= 48) && (currentPose.getY() <= 72)) {
                return "D6";
            } else if ((currentPose.getY() >= 72) && (currentPose.getY() <= 96)) {
                return "C6";
            }
        }

        if((currentPose.getX() >= 72) && (currentPose.getX() <= 96))  {
            if((currentPose.getY() >= 48) && (currentPose.getY() <= 72)) {
                return "D3";
            } else if ((currentPose.getY() >= 72) && (currentPose.getY() <= 96)) {
                return "C3";
            }
        }

        if((currentPose.getX() >= 96) && (currentPose.getX() <=120))  {
            if((currentPose.getY() >= 24) && (currentPose.getY() <= 48)) {
                return "E7";
            } else if ((currentPose.getY() >= 48) && (currentPose.getY() <= 72)) {
                return "D7";
            } else if ((currentPose.getY() >= 72) && (currentPose.getY() <= 96)) {
                return "C7";
            } else if ((currentPose.getY() >= 96) && (currentPose.getY() <= 120)) {
                return "B7";
            }
        }

        if(currentPose.getX() >= 120)  {
            if((currentPose.getY() >= 24) && (currentPose.getY() <= 48)) {
                return "E8";
            } else if ((currentPose.getY() >= 48) && (currentPose.getY() <= 72)) {
                return "D8";
            } else if ((currentPose.getY() >= 72) && (currentPose.getY() <= 96)) {
                return "C8";
            } else if ((currentPose.getY() >= 96) && (currentPose.getY() <= 120)) {
                return "B8";
            }
        }

        return "None";
    }
    Runnable testRunnable = () -> {
        System.out.println("test");
    };
}