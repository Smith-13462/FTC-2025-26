package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)
            .forwardZeroPowerAcceleration(-39.01846)
            .lateralZeroPowerAcceleration(-70.26015)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.01)) //manual tuning
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0.01))  //manual tuning
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(5, 0, 0.08, 0.01)) //manual tuning
            .useSecondaryHeadingPIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.0001, 0.6, 0.01)) //manual tuning
            .drivePIDFSwitch(15)  //manual tuning
            .centripetalScaling(0.0005)  //manual tuning
            ;
    public static PathConstraints pathConstraints = new PathConstraints(
                    0.995,
                    0.1,
                    0.1,
                    0.009,
                    250,
                    0.5,
                    10,
                    0.5
    );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(60.5527) //auto tuning
            .yVelocity(46.8720) //auto tuning
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {

        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }

    public static PathConstraints pickupPathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            100,
            0.4,
            10,
            0.4
    );

    public static PathConstraints turnConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            250,
            0.6,
            10,
            0.7
    );

}
