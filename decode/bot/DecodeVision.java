package org.firstinspires.ftc.teamcode.decode.bot;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConfig;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;
import org.firstinspires.ftc.teamcode.decode.common.DecodeUtil;

import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.AprilTagPosition.*;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.LimeLightPipeline.APRILTAG_PIPELINE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.BLUE_ALLIANCE;
import static org.firstinspires.ftc.teamcode.decode.common.DecodeConstants.TeamAllianceColor.RED_ALLIANCE;

public class DecodeVision
{
    HashMap<DecodeConstants.AprilTagPosition,Integer> tagPosToNum = new HashMap<DecodeConstants.AprilTagPosition,Integer>();
    HashMap<Integer, DecodeConstants.AprilTagPosition> tagNumToPos = new HashMap<Integer, DecodeConstants.AprilTagPosition>();
    HashMap<DecodeConstants.AprilTagPosition, Pose3D> tagNameToPose = new HashMap<DecodeConstants.AprilTagPosition, Pose3D>();
    private Telemetry telemetry;
    private boolean  updateLLMt2Pose;
    private Limelight3A limelight;
    private LLResult llResult = null;
    private boolean limelightAvailable = false;
    private Pose2D botPose = null;
    public String camDebugInfo;
    private double LL_X_ADJUSTMENT = 0, LL_Y_ADJUSTMENT = 0, LL_ANGLE_ADJUSTMENT = 0;
    IMU imu;
    DecodeConstants.LimeLightPipeline[] limeLightPipelines = {APRILTAG_PIPELINE};
    private static final Integer BLUE_TARGET_TAG_NUM = 20;
    private static final Integer RED_TARGET_TAG_NUM = 24;
    private static final Integer MOTIF_GPP_TAG_NUM = 21;
    private static final Integer MOTIF_PGP_TAG_NUM = 22;
    private static final Integer MOTIF_PPG_TAG_NUM = 23;
    private int visibleMotifTagNum  = 0;
    private DecodeConstants.TeamAllianceColor teamAllianceColor;
    private static DecodeConstants.Motif launchMotif = DecodeConstants.Motif.PGP_MOTIF;

    Pose3D LL_CAMERA_POSE =
               new Pose3D(new Position(DistanceUnit.CM, 0,0,0,0),
                    new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0,0));

    DecodeVision(HardwareMap hardwareMap, Telemetry telemetry, DecodeConstants.TeamAllianceColor teamAllianceColor){
        this.telemetry = telemetry;
        camDebugInfo = "";
        this.teamAllianceColor = teamAllianceColor;

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
            limelightAvailable = true;
        } catch (Exception e) {
            limelightAvailable = false;
        }
    }
    public void closeLLVision(){
        try {
            if(limelightAvailable) {
                limelight.stop();
                limelightAvailable = false;
            }
        } catch (Exception e) {
            limelightAvailable = false;
        }
    }
    public LLResult getLimeLightResult(){
        return llResult;
    }

    public String getCamDebugInfo(){
        return "cam inf="+ camDebugInfo;
    }
    public LLResult updateLimeLightResult(boolean updatePose, boolean updateMotif, DecodeConstants.TeamAllianceColor teamAllianceColor){
        LLStatus llStatus = null;
        llResult = null;

        if(!limelightAvailable) {
 //           camDebugInfo = "LL not available";
            return llResult;
        } else {
 //           camDebugInfo = "LL available";
        }
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
            //camDebugInfo += "llresult not null";

            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                    if(updateMotif) {
                        if (MOTIF_GPP_TAG_NUM.equals(fr.getFiducialId()) || MOTIF_PGP_TAG_NUM.equals(fr.getFiducialId()) ||
                                MOTIF_PPG_TAG_NUM.equals(fr.getFiducialId())) {
                            visibleMotifTagNum = fr.getFiducialId();
              //              camDebugInfo += "\n decode fiducial id: " + fr.getFiducialId();
                        } else {
              //              camDebugInfo += "\n non decode fiducial id: " + fr.getFiducialId();
                        }
                    }

                    if(updatePose){
                        if ((RED_ALLIANCE.equals(teamAllianceColor) && RED_TARGET_TAG_NUM.equals(fr.getFiducialId())) ||
                            (BLUE_ALLIANCE.equals(teamAllianceColor) && BLUE_TARGET_TAG_NUM.equals(fr.getFiducialId())) ||
                            ((teamAllianceColor == null) && (RED_TARGET_TAG_NUM.equals(fr.getFiducialId()) ||
                                    BLUE_TARGET_TAG_NUM.equals(fr.getFiducialId())))) {
                            botPose = new Pose2D(DistanceUnit.INCH, llResult.getBotpose().getPosition().toUnit(DistanceUnit.INCH).x,
                                    llResult.getBotpose().getPosition().toUnit(DistanceUnit.INCH).y,
                                    AngleUnit.DEGREES, llResult.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));
                        }
                    }
            }
        } else {
       //     camDebugInfo += "llresult null";
        }

        return llResult;
    }

    public DecodeConstants.Motif getDetectedMotif() {
        updateLimeLightResult(false, true, null);

        switch (visibleMotifTagNum) {
            case 21:
                this.launchMotif = DecodeConstants.Motif.GPP_MOTIF;
                break;
            case 22:
                this.launchMotif = DecodeConstants.Motif.PGP_MOTIF;
                break;
            case 23:
                this.launchMotif = DecodeConstants.Motif.PPG_MOTIF;
                break;
            default:
                this.launchMotif = DecodeConstants.Motif.PGP_MOTIF;
                break;
        }

        return this.launchMotif;
    }
    private void setFieldTargetPositions() {
        tagPosToNum.put(BLUE_TARGET_TAG,BLUE_TARGET_TAG_NUM);
        tagPosToNum.put(RED_TARGET_TAG,RED_TARGET_TAG_NUM);

        tagNumToPos.put(BLUE_TARGET_TAG_NUM, BLUE_TARGET_TAG);
        tagNumToPos.put(RED_TARGET_TAG_NUM, RED_TARGET_TAG);
    }

    public Integer getVisibleMotifTagNum() {

        return new Integer(visibleMotifTagNum);
    }

    public Pose2D getLLBotPose2D(DecodeConstants.TeamAllianceColor teamAllianceColor){
        botPose = null;
        Pose2D normalizedBotPose = null;
        if(!limelightAvailable) return null;

        updateLimeLightResult(true, true, teamAllianceColor);

        if(botPose != null) {
            if((llResult.getBotpose().getPosition().x == 0) && (llResult.getBotpose().getPosition().y == 0)) return null;

            double xCoordinate = botPose.getX(DistanceUnit.INCH);
            double yCoordinate = botPose.getY(DistanceUnit.INCH);

            double turnAngle = botPose.getHeading(AngleUnit.DEGREES);

            normalizedBotPose = new Pose2D(DistanceUnit.INCH, xCoordinate,
                    yCoordinate, AngleUnit.DEGREES, turnAngle);
        }

        return  normalizedBotPose;
    }

    public Pose getPedroPoseUsingTargetTag() {
        if(this.teamAllianceColor == null) return  null;

        Pose2D botRelativePose2D = getLLBotPose2D(teamAllianceColor);

        if(botRelativePose2D == null) return null;

        Pose botftcPose = PoseConverter.pose2DToPose(botRelativePose2D, FTCCoordinates.INSTANCE);
        Pose botPedroPose = botftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        return botPedroPose;
    }

    public Pose getPedroPoseUsingTargetTag1() {
        if(teamAllianceColor == null) return  null;

        Pose2D botRelativePose2D = getLLBotPose2D(teamAllianceColor);

        if(botRelativePose2D == null) return null;

        double reverseDriveAdjustment = 0;
        if(DecodeConfig.REVERSE_DRIVE_BOOLEAN){
            reverseDriveAdjustment = 180;
        }

        Pose botPedroPose = new Pose((botRelativePose2D.getX(DistanceUnit.INCH) * -1)  + 72 + LL_X_ADJUSTMENT
                ,(botRelativePose2D.getY(DistanceUnit.INCH) * -1) + 72 + LL_Y_ADJUSTMENT
               ,Math.toRadians(DecodeUtil.normalizeFieldAngle(DecodeUtil.normalizeFieldAngle(
                       DecodeUtil.normalizeFieldAngle(DecodeUtil.normalizeFieldAngle(botRelativePose2D.getHeading(AngleUnit.DEGREES)) + 180)
                + LL_ANGLE_ADJUSTMENT) + reverseDriveAdjustment)));

        return botPedroPose;
    }

    public DecodeConstants.Motif getMotif(){
        return this.launchMotif;
    }

    public String getVisionDisplayInfo(DecodeConstants.TeamAllianceColor teamAllianceColor) {
        String displayVisionInfo = "";
        if(!limelightAvailable) return displayVisionInfo;

        Pose2D botPose2D = getLLBotPose2D(teamAllianceColor);
        Pose botPedroPose = getPedroPoseUsingTargetTag();
        Pose botPedroPose1 = getPedroPoseUsingTargetTag1();

        if (botPose2D != null) {
            displayVisionInfo += "\n" + "Tag pose X= " + botPose2D.getX(DistanceUnit.INCH) +
                    " ,Y=" + botPose2D.getY(DistanceUnit.INCH) + " ,Angle=" + String.format("%6.1f", botPose2D.getHeading(AngleUnit.DEGREES));
        } else {
            displayVisionInfo += "\n" + "Tag pose not available";
        }

        if(botPedroPose != null){
            displayVisionInfo += "\n" + "Tag pedro pose X= " + botPedroPose.getX() +
                    " ,Y=" + botPedroPose.getY() + " ,Angle=" + String.format("%6.1f", Math.toDegrees(botPedroPose.getHeading()));
        } else {
            displayVisionInfo += "\n" + "Tag pedro pose not available";
        }

        if(botPedroPose1 != null){
            displayVisionInfo += "\n" + "Tag pedro pose1 X= " + botPedroPose1.getX() +
                    " ,Y=" + botPedroPose1.getY() + " ,Angle=" + String.format("%6.1f", Math.toDegrees(botPedroPose1.getHeading()));
        } else {
            displayVisionInfo += "\n" + "Tag pedro pose1 not available";
        }
        displayVisionInfo += ("\n" + "Motif = " + this.launchMotif);

        return displayVisionInfo;
    }
}