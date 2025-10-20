package org.firstinspires.ftc.teamcode.decode.bot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.AprilTagPosition.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.LimeLightPipeline.APRILTAG_PIPELINE;

public class DecodeVision
{
    HashMap<DecodeConstants.AprilTagPosition,Integer> tagPosToNum = new HashMap<DecodeConstants.AprilTagPosition,Integer>();
    HashMap<Integer, DecodeConstants.AprilTagPosition> tagNumToPos = new HashMap<Integer, DecodeConstants.AprilTagPosition>();
    HashMap<DecodeConstants.AprilTagPosition, Pose3D> tagNameToPose = new HashMap<DecodeConstants.AprilTagPosition, Pose3D>();
    HashMap<DecodeConstants.FieldTarget, Pose2D> fieldTargetPoseMap = new HashMap<DecodeConstants.FieldTarget, Pose2D>();
    private boolean  updateLLMt2Pose;
    private Limelight3A limelight;
    private LLResult llResult = null;
    private boolean llAvailable = false;
    private Pose2D botPose = null;
    public String camDebugInfo;
    IMU imu;
    DecodeConstants.LimeLightPipeline[] limeLightPipelines = {APRILTAG_PIPELINE};
    private static final Integer BLUE_TARGET_TAG_NUM = 20;
    private static final Integer RED_TARGET_TAG_NUM = 24;
    private static final Integer MOTIF_GPP_TAG_NUM = 21;
    private static final Integer MOTIF_PGP_TAG_NUM = 22;
    private static final Integer MOTIF_PPG_TAG_NUM = 23;
    private Integer visibleMotifTagNum = 0;

    Pose3D BLUE_TARGET_TAG_POSE =
            new Pose3D(new Position(DistanceUnit.CM, 0,0,0,0),
                    new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0,0));
    Pose3D RED_TARGET_TAG_POSE =
            new Pose3D(new Position(DistanceUnit.CM, 0,0,0,0),
                    new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0,0));
     Pose3D LL_CAMERA_POSE =
               new Pose3D(new Position(DistanceUnit.CM, 0,0,0,0),
                    new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0,0));

    DecodeVision(HardwareMap hardwareMap){
        camDebugInfo = "";

        if(limeLightPipelines.length > 0){
            initLimeLightVision(hardwareMap, limeLightPipelines);
        }
        this.updateLLMt2Pose = false;

        setFieldTargetPositions();
    }
    public void initLimeLightVision(HardwareMap hardwareMap, DecodeConstants.LimeLightPipeline[] limeLightPipelines){
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (updateLLMt2Pose) {
                imu = hardwareMap.get(IMU.class, "imu");
            }
            limelight.pipelineSwitch(0);
            limelight.start();
            llAvailable = true;
        } catch (Exception e) {
            llAvailable = false;
        }
    }
    public void closeLLVision(){
        try {
            if(llAvailable) {
                limelight.stop();
                llAvailable = false;
            }
        } catch (Exception e) {
            llAvailable = false;
        }
    }
    public LLResult getLimeLightResult(){
        return llResult;
    }
    public LLResult updateLimeLightResult(boolean updatePose, boolean updateMotif){
        LLStatus llStatus = null;
        llResult = null;

        if(!llAvailable) return llResult;
        llStatus  = limelight.getStatus();

        if(updateLLMt2Pose) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        }

        try {
            llResult = limelight.getLatestResult();
        }catch(Exception e){
            //do nothing
        }
        if (llResult != null) {
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                    if(updateMotif) {
                        if (MOTIF_GPP_TAG_NUM.equals(fr.getFiducialId()) || MOTIF_PGP_TAG_NUM.equals(fr.getFiducialId()) ||
                                MOTIF_PPG_TAG_NUM.equals(fr.getFiducialId())) {
                            visibleMotifTagNum = fr.getFiducialId();
                        }
                    }

                    if(updatePose){
                        if (RED_TARGET_TAG_NUM.equals(fr.getFiducialId()) || BLUE_TARGET_TAG_NUM.equals(fr.getFiducialId())) {
                            botPose = new Pose2D(DistanceUnit.INCH, llResult.getBotpose().getPosition().toUnit(DistanceUnit.INCH).x,
                                    llResult.getBotpose().getPosition().toUnit(DistanceUnit.INCH).y,
                                    AngleUnit.DEGREES, llResult.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));
                        }
                    }
            }
        }

        return llResult;
    }

    public DecodeConstants.Motif getDetectedMotif() {
        updateLimeLightResult(false, true);

        switch (visibleMotifTagNum) {
            case 21:
                return DecodeConstants.Motif.GPP_MOTIF;
            case 22:
                return DecodeConstants.Motif.PGP_MOTIF;
            case 23:
                return DecodeConstants.Motif.PPG_MOTIF;
            default:
                return DecodeConstants.Motif.UNKNOWN_MOTIF;
        }
    }
    private void setFieldTargetPositions() {
        tagPosToNum.put(BLUE_TARGET_TAG,BLUE_TARGET_TAG_NUM);
        tagPosToNum.put(RED_TARGET_TAG,RED_TARGET_TAG_NUM);

        tagNumToPos.put(BLUE_TARGET_TAG_NUM, BLUE_TARGET_TAG);
        tagNumToPos.put(RED_TARGET_TAG_NUM, RED_TARGET_TAG);

        tagNameToPose.put(BLUE_TARGET_TAG,BLUE_TARGET_TAG_POSE);
        tagNameToPose.put(RED_TARGET_TAG,RED_TARGET_TAG_POSE);
    }

    public Integer getVisibleMotifTagNum() {
        return visibleMotifTagNum;
    }

    public Pose2D getTargetPoseRelativeToBot(DecodeConstants.FieldTarget target) {
        double forwardDistance = 0, rightDistance = 0, turnAngle = 0;
        Pose2D  relativePose = null ;

        botPose = getLLBotPose();
        Pose2D targetPose = fieldTargetPoseMap.get(target);

        if((botPose == null) || (targetPose == null)){
            return null;
        }

        forwardDistance = targetPose.getX(DistanceUnit.INCH) - botPose.getX(DistanceUnit.INCH);
        rightDistance = targetPose.getY(DistanceUnit.INCH) - botPose.getY(DistanceUnit.INCH);
        turnAngle = targetPose.getHeading(AngleUnit.DEGREES) - botPose.getHeading(AngleUnit.DEGREES);
        turnAngle = normalizeFTCFieldAngle(turnAngle);

        relativePose =  new Pose2D(DistanceUnit.INCH, forwardDistance,rightDistance,
                AngleUnit.DEGREES, turnAngle);

        return relativePose;
    }
    public Pose2D getLLBotPose(){
        botPose = null;
        Pose2D normalizedBotPose = null;
        if(!llAvailable) return null;

        updateLimeLightResult(true, true);

        if(botPose != null) {
            if((llResult.getBotpose().getPosition().x == 0) && (llResult.getBotpose().getPosition().y == 0)) return null;

            double xCoordinate = botPose.getX(DistanceUnit.INCH);
            double yCoordinate = botPose.getY(DistanceUnit.INCH);

            double turnAngle = botPose.getHeading(AngleUnit.DEGREES);
            turnAngle = normalizeFTCFieldAngle(turnAngle);
            turnAngle = turnAngle - LL_CAMERA_POSE.getOrientation().getYaw(AngleUnit.DEGREES);
            turnAngle = normalizeFTCFieldAngle(turnAngle);

            normalizedBotPose = new Pose2D(DistanceUnit.INCH, xCoordinate,
                    yCoordinate, AngleUnit.DEGREES, turnAngle);

        }
        return  normalizedBotPose;
    }
    public double normalizeFTCFieldAngle(double angle){
        double normalizedAngle = angle;

        if(normalizedAngle > 180) {
            normalizedAngle = (360 - normalizedAngle) * -1;
        } else if(normalizedAngle < -180) {
            normalizedAngle = 360 + normalizedAngle;
        }

        return normalizedAngle;
    }
}