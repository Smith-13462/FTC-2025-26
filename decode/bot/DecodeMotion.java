package org.firstinspires.ftc.teamcode.decode.bot;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.common.DecodeUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConfig.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.OP_MODE.AUTO_OP_MODE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.RED_ALLIANCE;

public class DecodeMotion {
    HardwareMap hardwareMap;
    GoBildaPinpointDriver pinpoint;
    boolean pinpointAvailable = false, pedropathDrive = false;
    private Follower pedroPathFollower, pickupPathFollower;
    Telemetry telemetry = null;
    private Pose startPose;
    private DecodeBot owlsBotDecode;
    private static Pose lastKnownAutoBotPose = null;
    private static ElapsedTime timeSinceLastknownAutoBotPose;
    private boolean botPositionOnHold = false;
    private double acceptableDelta = 1;
    private OP_MODE opMode;

    static {
        timeSinceLastknownAutoBotPose = new ElapsedTime();
    }
    DecodeMotion(HardwareMap hardwareMap, DecodeBot owlsBotDecode, boolean automatedDrive, boolean recalibratePinpoint,
                 Pose pStartPose, Telemetry telemetry, OP_MODE opMode){
        this.hardwareMap = hardwareMap;
        this.pedropathDrive = automatedDrive;
        this.telemetry = telemetry;
        this.owlsBotDecode = owlsBotDecode;
        this.opMode = opMode;

        if(pStartPose == null) {
            this.startPose = new Pose(0,0, 0);
        } else {
            this.startPose = new Pose(pStartPose.getX(), pStartPose.getY(), pStartPose.getHeading());
        }

        //pinpoint
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpointAvailable = true;
            if(recalibratePinpoint) {
                configurePinpoint(recalibratePinpoint);
                pinpoint.update();
            }
        } catch (Exception e){
            pinpointAvailable = false;
            this.pedropathDrive = false;
        }

        if(pedropathDrive) {
            pedroPathFollower = Constants.createFollower(hardwareMap);
            pedroPathFollower.setStartingPose(new Pose(this.startPose.getX(), this.startPose.getY(), this.startPose.getHeading()));
            pedroPathFollower.update();
            lastKnownAutoBotPose = pedroPathFollower.getPose();
            if(AUTO_OP_MODE.equals(opMode)) {
                timeSinceLastknownAutoBotPose.reset();
            }

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
        while(hold.time() < 2) {}
        telemetry.addLine("pinpoint recalibration done");
        //telemetry.update();
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
        pedroPathFollower.update();
        while ((pedroPathFollower.isBusy() || pedroPathFollower.getDistanceRemaining() > 0 || pedroPathFollower.isTurning())
                && (timer.time() < 1)) {
            pedroPathFollower.update();
            telemetry.addLine("Turning off bot position hold...");
            telemetry.update();
        }

        pedroPathFollower.breakFollowing();
        lastKnownAutoBotPose = pedroPathFollower.getPose();
        if(AUTO_OP_MODE.equals(opMode)) {
            timeSinceLastknownAutoBotPose.reset();
        }

        this.botPositionOnHold = false;
    }
    public void moveLinearOrTurnToLaunch(TeamAllianceColor teamAllianceColor, double left, double forward, boolean hold){
        double turnAngle = 0, targetAngle = 0;
        boolean leftTurn = false;
        ElapsedTime timer = new ElapsedTime();

        pedroPathFollower.update();
        Pose currentBotPosePedro = new Pose(pedroPathFollower.getPose().getX(), pedroPathFollower.getPose().getY(),
                pedroPathFollower.getPose().getHeading());
        turnAngle = DecodeUtil.getLaunchHeadingAdjustment(teamAllianceColor,
                currentBotPosePedro, left, forward);
        targetAngle = DecodeUtil.normalizeFieldAngle(Math.toDegrees(DecodeUtil.getLaunchHeading(teamAllianceColor, currentBotPosePedro)));

        if((left == 0) && (forward == 0)) {
            if(Math.toDegrees(Math.abs(turnAngle)) > 0.5) {
                leftTurn = (Math.toDegrees(turnAngle) > 0) ? true : false;
                pedroPathFollower.setMaxPower(0.6);
                pedroPathFollower.setConstraints(Constants.turnConstraints);
                pedroPathFollower.turnDegrees(Math.abs(Math.toDegrees(turnAngle)), leftTurn);
                pedroPathFollower.update();

                timer.reset();
                while (
                    //(pedroPathFollower.isBusy() || (pedroPathFollower.getDistanceRemaining() > 0) || pedroPathFollower.isTurning())
                        (Math.abs(DecodeUtil.normalizeFieldAngle(targetAngle - DecodeUtil.normalizeFieldAngle(Math.toDegrees(pedroPathFollower.getPose().getHeading())))) > 0.5 ) &&
                                (timer.time() < 3.5)) {
                    pedroPathFollower.update();
                    telemetry.addLine("Turning " + (leftTurn ? "left " : "right ") + Math.toDegrees(Math.abs(turnAngle)) + " degrees.");
                    telemetry.update();
                }
                pedroPathFollower.breakFollowing();
                this.botPositionOnHold = false;
            } else {
                telemetry.addLine("No turn needed. Already at an angle to launch...");
                telemetry.update();
            }
        } else {
            PathChain moveAndTurnPathChain =  pedroPathFollower.pathBuilder()
                    .addPath(new BezierLine(currentBotPosePedro,
                            new Pose(currentBotPosePedro.getX() + forward,currentBotPosePedro.getY() + left,
                                    Math.toRadians(DecodeUtil.normalizeFieldAngle(DecodeUtil.normalizeFieldAngle(Math.toDegrees(currentBotPosePedro.getHeading())) + turnAngle)))
                    ))
                    .setLinearHeadingInterpolation(currentBotPosePedro.getHeading(),
                            Math.toRadians(DecodeUtil.normalizeFieldAngle(DecodeUtil.normalizeFieldAngle(Math.toDegrees(currentBotPosePedro.getHeading())) + turnAngle)),
                            0.8)
                    .build();

            pedroPathFollower.followPath(moveAndTurnPathChain, hold);
            pedroPathFollower.update();

            timer.reset();
            while ((pedroPathFollower.isBusy() || (pedroPathFollower.getDistanceRemaining() > 0) || pedroPathFollower.isTurning()) &&
                 //   (Math.abs(DecodeUtil.normalizeFieldAngle(targetAngle - DecodeUtil.normalizeFieldAngle(Math.toDegrees(pedroPathFollower.getPose().getHeading())))) > 0 ) &&
                            (timer.time() < 4)) {
                pedroPathFollower.update();
                telemetry.addLine("Turning " + (leftTurn ? "left " : "right ") + Math.toDegrees(Math.abs(turnAngle)) + " degrees.");
                //           telemetry.addLine("Current Angle = " + DecodeUtil.normalizeFieldAngle(Math.toDegrees(pedroPathFollower.getPose().getHeading())));
                telemetry.update();
            }
            pedroPathFollower.breakFollowing();
        }


        lastKnownAutoBotPose = pedroPathFollower.getPose();
        if(AUTO_OP_MODE.equals(opMode)) {
            timeSinceLastknownAutoBotPose.reset();
        }
    }

    public void travelToLaunchOrPickup(ArrayList<Pose> travelPathControlPoints, Runnable parallelAction){

        ElapsedTime timer = new ElapsedTime();

        if(travelPathControlPoints.size() > 2) {
            PathChain travelPathChain;
            if (parallelAction != null) {
                travelPathChain = pedroPathFollower.pathBuilder()
                        .addPath(new BezierCurve(travelPathControlPoints))
                        .setLinearHeadingInterpolation(travelPathControlPoints.get(0).getHeading(),
                                travelPathControlPoints.get(travelPathControlPoints.size() - 1).getHeading(), 0.5)
                        .addParametricCallback(0.1, parallelAction)
                        .build();
            } else {
                travelPathChain = pedroPathFollower.pathBuilder()
                            .addPath(new BezierCurve(travelPathControlPoints))
                            .setLinearHeadingInterpolation(travelPathControlPoints.get(0).getHeading(),
                                    travelPathControlPoints.get(travelPathControlPoints.size() - 1).getHeading(), 0.5)
                            .build();
            }

            pedroPathFollower.setMaxPower(0.9);
            pedroPathFollower.setConstraints(Constants.pathConstraints);
            pedroPathFollower.followPath(travelPathChain, true);
            timer.reset();
            while((pedroPathFollower.isBusy() || pedroPathFollower.isTurning() || (pedroPathFollower.getDistanceRemaining() > 0))
            && (timer.time() < 5)){
                pedroPathFollower.update();
            };
            pedroPathFollower.breakFollowing();
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

            pedroPathFollower.setMaxPower(0.9);
            pedroPathFollower.setConstraints(Constants.pathConstraints);
            pedroPathFollower.followPath(travelPathChain, true);
            timer.reset();
            while((pedroPathFollower.isBusy() || pedroPathFollower.isTurning() || (pedroPathFollower.getDistanceRemaining() > 0))
                  && (timer.time() < 5)){
                pedroPathFollower.update();
            };
            pedroPathFollower.breakFollowing();
        }

        lastKnownAutoBotPose = pedroPathFollower.getPose();
        if(AUTO_OP_MODE.equals(opMode)) {
            timeSinceLastknownAutoBotPose.reset();
        }

    }

    public void moveStraightToPickup(TeamAllianceColor teamAllianceColor,
                                         FieldPosition pickupPosition, int pickupSequence) {
        Pose pickupStartPose = DecodeConfig.getFieldPositionPose(pickupPosition);
        if(pickupStartPose == null) return;

        double teamColorAdj = 1;
        if(RED_ALLIANCE.equals(teamAllianceColor)){
            teamColorAdj = -1;
        }

        ArrayList<Pose> pickupPathControlPoints =  new ArrayList<>();
        Pose currentPose = getUpdatedBotPose();
        Pose pickupIntermediatePose = new Pose (pickupStartPose.getX(),
                (pickupStartPose.getY() + ((5 * pickupSequence * teamColorAdj) / 2)), pickupStartPose.getHeading());
        Pose pickupEndPose = new Pose (pickupStartPose.getX(),
                (pickupStartPose.getY() + (5 * pickupSequence * teamColorAdj)), pickupStartPose.getHeading());

        pickupPathControlPoints.add(0, new Pose (currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
 //       pickupPathControlPoints.add(0, pickupStartPose);
        pickupPathControlPoints.add(1, pickupIntermediatePose);
        pickupPathControlPoints.add(2, pickupEndPose);

        Path pickupPath = new Path(new BezierCurve(pickupPathControlPoints), Constants.pickupPathConstraints);
        PathChain pickupPathChain;
        pickupPathChain = pedroPathFollower.pathBuilder()
                .addPath(pickupPath)
                .setLinearHeadingInterpolation(pickupPathControlPoints.get(0).getHeading(),
                        pickupPathControlPoints.get(pickupPathControlPoints.size() - 1).getHeading(), 0.1)
                .build();

        pedroPathFollower.setMaxPower(0.20);
        pedroPathFollower.followPath(pickupPathChain, true);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while((pedroPathFollower.isBusy() || pedroPathFollower.isTurning() || (pedroPathFollower.getDistanceRemaining() > 0))
                && (timer.time() < 3)){
            pedroPathFollower.update();
        };
        pedroPathFollower.breakFollowing();

        lastKnownAutoBotPose = pedroPathFollower.getPose();
        if(AUTO_OP_MODE.equals(opMode)) {
            timeSinceLastknownAutoBotPose.reset();
        }
    }

    public void travelToLaunchOrPickup(ArrayList<Pose> travelPathControlPoints, boolean retry, Runnable parallelAction) {
        travelToLaunchOrPickup(travelPathControlPoints, parallelAction);

        if(retry && (travelPathControlPoints.size() > 0) &&
            !isBotPoseWithinAcceptableDelta(travelPathControlPoints.get(travelPathControlPoints.size() - 1))) {
            ArrayList<Pose> retryControlPoints = new ArrayList<>();
            Pose currentPose = getUpdatedBotPose();
            retryControlPoints.add(0, currentPose);
            retryControlPoints.add(1, travelPathControlPoints.get(travelPathControlPoints.size() - 1));
            travelToLaunchOrPickup(retryControlPoints, null);
        }
    }

    public void travelToLaunchOrPickup(TravelPathShape pathShape, Pose toPose, boolean retry, Runnable parallelAction){

        ArrayList<Pose> travelPathControlPoints =  new ArrayList<>();

        Pose currentPose = getUpdatedBotPose();

        travelPathControlPoints.add(0, new Pose (currentPose.getX(), currentPose.getY(), currentPose.getHeading()));

        if(pathShape.equals(TravelPathShape.CURVE)) {
            Pose intermediatePoint = new Pose((currentPose.getX() + toPose.getX()) / 2,
                    (currentPose.getY() + toPose.getY()) / 2,  toPose.getHeading());
            travelPathControlPoints.add(1, intermediatePoint);
            travelPathControlPoints.add(2, toPose);
        } else {
            travelPathControlPoints.add(1, toPose);
        }

        travelToLaunchOrPickup(travelPathControlPoints, retry, parallelAction);
    }

    public void travelToLaunchOrPickup(TravelPathShape pathShape, FieldPosition toPosition, boolean retry, Runnable parallelAction){

        Pose toPose = owlsBotDecode.getConfig().getFieldPositionPose(toPosition);

        if(toPose != null) {
            travelToLaunchOrPickup(pathShape, toPose, retry, parallelAction);
        }
    }

    public void travelToLaunchOrPickup(TravelPathShape pathShape, FieldPosition fromPosition, FieldPosition toPosition, boolean retry, Runnable parallelAction) {
        ArrayList<Pose> travelPathControlPoints =  owlsBotDecode.getConfig().getCurvePoints(fromPosition, toPosition);

        if((travelPathControlPoints == null) || (travelPathControlPoints.size() == 1)) {
            if(toPosition != null) {
                travelToLaunchOrPickup(pathShape, toPosition, retry, parallelAction);
            }
            return;
        }

        if((pathShape.equals(TravelPathShape.CURVE)) && (travelPathControlPoints.size() == 2)) {
            Pose intermediatePoint = new Pose((travelPathControlPoints.get(0).getX() + travelPathControlPoints.get(1).getX()) / 2,
                    (travelPathControlPoints.get(0).getY() + travelPathControlPoints.get(1).getY()) / 2,
                    travelPathControlPoints.get(1).getHeading());
            travelPathControlPoints.add(2, travelPathControlPoints.get(1));
            travelPathControlPoints.add(1, intermediatePoint);
        }

        if(!isBotPoseWithinAcceptableDelta(travelPathControlPoints.get(0))) {
            travelPathControlPoints.add(0, new Pose(lastKnownAutoBotPose.getX(), lastKnownAutoBotPose.getY(), lastKnownAutoBotPose.getHeading()));
        }

        travelToLaunchOrPickup(travelPathControlPoints, retry, parallelAction);
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

        lastKnownAutoBotPose = pedroPathFollower.getPose();
        if(AUTO_OP_MODE.equals(opMode)) {
            timeSinceLastknownAutoBotPose.reset();
        }
    }

    public Pose getUpdatedBotPose() {
        pedroPathFollower.update();
        lastKnownAutoBotPose = pedroPathFollower.getPose();
        if(AUTO_OP_MODE.equals(opMode)) {
            timeSinceLastknownAutoBotPose.reset();
        }
        return lastKnownAutoBotPose;
    }

    public static Pose getLastKnownAutoBotPose(){
        return lastKnownAutoBotPose;
    }
    public static ElapsedTime getTimeSinceLastknownAutoBotPose(){
        return timeSinceLastknownAutoBotPose;
    }
    public String getBotPositionDisplayInfo(TeamAllianceColor teamAllianceColor, boolean displayPosition) {
        pedroPathFollower.update();

        Pose latestPedroPose = new Pose(pedroPathFollower.getPose().getX(),
                pedroPathFollower.getPose().getY(), pedroPathFollower.getPose().getHeading());
        String launchPositionName = getLaunchPositionName(latestPedroPose);
        String displayInfo = "";

        if(!pinpointAvailable) return displayInfo;

        double turnAngle = DecodeUtil.getLaunchHeadingAdjustment(teamAllianceColor,
                latestPedroPose, 0, 0);

        try {
               if(displayPosition) {
                   /*
                   displayInfo += "\n" + "Bot Start Position:"
                           + String.format("%6.1f", startPose.getX())
                           + "," + String.format("%6.1f", startPose.getY())
                           + "," + String.format("%6.1f", Math.toDegrees(startPose.getHeading()));

                    */
                   displayInfo += "\n" + "Bot Position:"
                           + String.format("%6.1f", latestPedroPose.getX())
                           + "," + String.format("%6.1f", latestPedroPose.getY())
                           + "," + String.format("%6.1f", Math.toDegrees(latestPedroPose.getHeading()));
               }
               displayInfo += "\nBot launch position: " + launchPositionName;
               displayInfo += "\nTurn needed to launch is" + String.format("%6.1f ", Math.toDegrees(Math.abs(turnAngle))) + ((turnAngle > 0) ? "degrees left" : "degrees right");

            if (this.botPositionOnHold) {
                displayInfo +="\nPress DPad DOWN to resume steering...";
            }
        } catch (Exception ex) {
               displayInfo += "\n!!!Bot Position not available!!!";
        }

        return displayInfo;
    }

    public String getLaunchPositionName(Pose currentPose) {

        if(currentPose == null) return "None";

        if(currentPose.getX() <= 24)  {
            if((currentPose.getY() >= 48) && (currentPose.getY() <= 72)) {
                return "D1";
            } else if ((currentPose.getY() >= 72) && (currentPose.getY() <= 96)) {
                return "C1";
            }
        }

        if((currentPose.getX() >= 72) && (currentPose.getX() <= 96))  {
            if((currentPose.getY() >= 48) && (currentPose.getY() <= 72)) {
                return "D4";
            } else if ((currentPose.getY() >= 72) && (currentPose.getY() <= 96)) {
                return "C4";
            }
        }

        if((currentPose.getX() >= 96) && (currentPose.getX() <=120))  {
            if((currentPose.getY() >= 24) && (currentPose.getY() <= 48)) {
                return "E5";
            } else if ((currentPose.getY() >= 48) && (currentPose.getY() <= 72)) {
                return "D5";
            } else if ((currentPose.getY() >= 72) && (currentPose.getY() <= 96)) {
                return "C5";
            } else if ((currentPose.getY() >= 96) && (currentPose.getY() <= 120)) {
                return "B5";
            }
        }

        if(currentPose.getX() >= 120)  {
            if((currentPose.getY() >= 24) && (currentPose.getY() <= 48)) {
                return "E6";
            } else if ((currentPose.getY() >= 48) && (currentPose.getY() <= 72)) {
                return "D6";
            } else if ((currentPose.getY() >= 72) && (currentPose.getY() <= 96)) {
                return "C6";
            } else if ((currentPose.getY() >= 96) && (currentPose.getY() <= 120)) {
                return "B6";
            }
        }

        return "None";
    }
}