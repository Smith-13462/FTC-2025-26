package org.firstinspires.ftc.teamcode.decode.bot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DOwlsBotBase<orientation> {
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
    public Telemetry telemetry = null;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double leftRearSpeed = 0;
    private double rightRearSpeed = 0;
    private boolean motorsAvailable = true;

    //NOT USED - START
    public Position robotCurrentPos;
    private double turnSpeed = 0;
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;

    private int leftTarget = 0;
    private int rightTarget = 0;
    private int leftRearTarget = 0;
    private int rightRearTarget = 0;

    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor frontEncoder = null;

    // Autonomous Drive and Turn Speed and Lift Motor Speed
    public static final double AUTONOMOUS_DRIVE_SPEED = 1.0;
    public static final double AUTONOMOUS_STRAFE_SPEED = 1.0;
    // Speeds are in Percentage, means 0.5 s 50% of max value
    //static final double DRIVE_SPEED =  0.5;
    static final double TURN_SPEED =  0.2;
    //static final double LIFT_SPEED = 0.7;

    // Robot width and Length, and wheel radius
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    static final double ROBOT_WIDTH_INCHES = 14.4;
    static final double ROBOT_LENGTH_INCHES = 11.4;
    static final double ROBOT_HEIGHT_INCHES = 4.0;
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double  MOTOR_MULTIPLIER = 1;
    static final double  WHEELS_DIAMETER_INCHES = 4;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * MOTOR_MULTIPLIER)/ (WHEELS_DIAMETER_INCHES *3.1415);
    static final double  COUNTS_PER_MOTOR_REV_ULTRA = 28;
    static final double STRAFE_FACTOR =1.3;  // Validate
    static final double COUNTS_PER_INCH_STRAFE = STRAFE_FACTOR*(COUNTS_PER_MOTOR_REV * MOTOR_MULTIPLIER)/ (WHEELS_DIAMETER_INCHES * 3.1415);
    static final double MOTOR_MAX_RPM = 312;
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    public DigitalChannel redleft, redright,greenleft, greenright;
    public Orientation angles;
    public IMU imu;

    DOwlsBotBase(){

    }
    public void Initialize_TeleOp_Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        Initialize_Common(hardwareMap, false);
        SetPower_TeleOp_Robot(leftSpeed, rightSpeed, leftRearSpeed, rightRearSpeed);
    }
    public void Initialize_Autonomous_Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        Initialize_Common(hardwareMap, true);
    }

    public void Initialize_Common(HardwareMap hardwareMap, boolean autonomous) {

        try {
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            leftRear = hardwareMap.get(DcMotor.class, "leftRear");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            rightRear = hardwareMap.get(DcMotor.class, "rightRear");

            if(!autonomous) {
                leftFront.setDirection(DcMotor.Direction.REVERSE);
                leftRear.setDirection(DcMotor.Direction.REVERSE);
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                rightRear.setDirection(DcMotor.Direction.FORWARD);
            }

            motorsAvailable = true;
        } catch(Exception e){
            motorsAvailable = false;
            telemetry.addData("Problem initialing wheel motors \n" , e.toString());
            telemetry.update();
        }

        //imu init
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        //resetHeading();
    }
    public void SetPower_TeleOp_Robot(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        if(motorsAvailable) {
            //2025 added minus multiplier to align direction with autonomous
            leftFront.setPower(leftFrontPower * -1);
            rightFront.setPower(rightFrontPower * -1);
            leftRear.setPower(leftRearPower * -1);
            rightRear.setPower(rightRearPower * -1);
        }
    }
    public void manualSteer(double drive, double turn, double strafe1, double strafe2, double powerMultiplier) {

        double leftFrontPower = 0, rightFrontPower = 0, leftRearPower = 0, rightRearPower = 0;
/*
        if ((strafe1 != 0) || (strafe2 != 0)) {
            if (strafe1 != 0) {
                leftFrontPower = strafe1 * powerMultiplier;
                rightFrontPower = -strafe1 * powerMultiplier;
                leftRearPower = -strafe1 * powerMultiplier;
                rightRearPower = strafe1 * powerMultiplier;
            } else {
                leftFrontPower = -strafe2 * powerMultiplier;
                rightFrontPower = strafe2 * powerMultiplier;
                leftRearPower = strafe2 * powerMultiplier;
                rightRearPower = -strafe2 * powerMultiplier;
            }
 */
        strafe1 *= -1;
        if (strafe1 != 0) {
            leftFrontPower = strafe1 * powerMultiplier;
            rightFrontPower = -strafe1 * powerMultiplier;
            leftRearPower = -strafe1 * powerMultiplier;
            rightRearPower = strafe1 * powerMultiplier;
        } else {
            leftFrontPower = (Range.clip(drive - turn, -1.0, 1.0)) * powerMultiplier;
            rightFrontPower = (Range.clip(drive + turn, -1.0, 1.0)) * powerMultiplier;
            leftRearPower = leftFrontPower;
            rightRearPower = rightFrontPower;
        }
        SetPower_TeleOp_Robot(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }
















    // NOT USED:

   public Position getCurrentRobotPosition() {
        return robotCurrentPos;
    }

    public void setCurrentRobotPosition(Position pos) {
        robotCurrentPos = pos;
    }
    public void calculateRobotNewDisplacement(Position pos) {
        //Calculate new robot position and then set the new position
        setCurrentRobotPosition(pos);
    }

    public void Initialize_Start_Position_Autonomous()
    {

    }
    public void SetPower_AutonomousOp_Robot(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    public void moveRight_Autonomous_Robot(double distance) {
        strafeLeft(AUTONOMOUS_STRAFE_SPEED, -distance, robotHeading);
    }

    public void moveRight_Autonomous_Robot(double distance, double motorspeed) {
        strafeLeft(motorspeed, -distance, robotHeading);

    }
    public void moveLeft_Autonomous_Robot(double distance) {
        strafeLeft(AUTONOMOUS_STRAFE_SPEED, distance, robotHeading);
    }

    public void moveLeft_Autonomous_Robot(double distance, double motorspeed) {
        strafeLeft(motorspeed, distance, robotHeading);

    }
    public void moveForward_Autonomous_Robot(double distance) {
        driveStraight(AUTONOMOUS_DRIVE_SPEED, distance, 0);
    }

    public void moveForward_Autonomous_Robot(double distance, double motorspeed) {
        driveStraight(motorspeed, distance, 0);
    }

    public void moveReverse_Autonomous_Robot(double distance) {
        driveStraight(AUTONOMOUS_DRIVE_SPEED, -distance, 0);
    }

    public void moveReverse_Autonomous_Robot(double distance, double motorspeed) {
        driveStraight(motorspeed, -distance, 0);
    }

    public void turnClock_Autonomous_Robot(double angledegrees) {
        turnToHeading(TURN_SPEED, -angledegrees);
        holdHeading(TURN_SPEED, -angledegrees, 0.9);
    }

    public void turnClock_Autonomous_Robot(double angledegrees, double motorspeed) {
        turnToHeading(motorspeed, -angledegrees);
        holdHeading(motorspeed, -angledegrees, 0.9);
    }
    public void turnAntiClock_Autonomous_Robot(double angledegrees) {
        turnToHeading(TURN_SPEED, angledegrees);
        holdHeading(TURN_SPEED, angledegrees, 0.9);
    }

    public void turnAntiClock_Autonomous_Robot(double angledegrees, double motorspeed) {
        turnToHeading(motorspeed, angledegrees);
        holdHeading(motorspeed, angledegrees, 0.9);
    }

    //*
    // * ====================================================================================================
    // * Driving "Helper" functions are below this line.
    // * These provide the high and low level methods that handle driving straight and turning.
    // * ====================================================================================================
    //
    // **********  HIGH Level driving functions.  ********************
    //**
   //  * Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
   ///  * Move will stop if either of these conditions occur:
   //  * 1) Move gets to the desired position
   //  * 2) Driver stops the opmode running.
   //  *
   //  * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
   //  * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
   //  * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
   //  *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
   //  *                      If a relative angle is required, add/subtract from the current robotHeading.
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        resetHeading();
        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        leftTarget = leftFront.getCurrentPosition() + moveCounts;
        rightTarget = rightFront.getCurrentPosition() + moveCounts;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftFront.setTargetPosition(leftTarget);
        leftRear.setTargetPosition(leftTarget);
        rightFront.setTargetPosition(rightTarget);
        rightRear.setTargetPosition(rightTarget);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed, 0);

        //telemetry.addData("VisiblePath  ", "Before While Loop");
        //telemetry.addData("Target Pos L:R",  "%f:%7d",      COUNTS_PER_INCH,  moveCounts);
        //telemetry.update();

        // keep looping while we are still active, and BOTH motors are running.
        while (
                (leftFront.isBusy() && rightFront.isBusy())) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(driveSpeed, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(true);
            //sleep(100);
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //**
    // * Method to drive in a straight line left or right , on a fixed compass heading (angle), based on encoder counts.
    // * Move will stop if either of these conditions occur:
    // * 1) Move gets to the desired position
    /// * 2) Driver stops the opmode running.
    // *
    // * @param maxDriveSpeed MAX Speed for left/right motion (range 0 to +1.0) .
    // * @param distance      Distance (in inches) to move from current position.  Negative distance means move right.
    // * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
    // *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    // *                      If a relative angle is required, add/subtract from the current robotHeading.
    //
    public void strafeLeft(double maxDriveSpeed,
                           double distance,
                           double heading) {

        resetHeading();

        // Determine new target position, and pass to motor controller

        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        leftTarget = leftFront.getCurrentPosition() - moveCounts;
        leftRearTarget = leftRear.getCurrentPosition() + moveCounts;
        rightTarget = rightFront.getCurrentPosition() + moveCounts;
        rightRearTarget = rightRear.getCurrentPosition() - moveCounts;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftFront.setTargetPosition(leftTarget);
        leftRear.setTargetPosition(leftRearTarget);
        rightFront.setTargetPosition(rightTarget);
        rightRear.setTargetPosition(rightRearTarget);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        strafeRobot(maxDriveSpeed, 0);

        // keep looping while we are still active, and BOTH motors are running.
        while ((leftFront.isBusy() && rightFront.isBusy())) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            strafeRobot(driveSpeed, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(true);
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //**
    // * Method to spin on central axis to point in a new direction.
    // * Move will stop if either of these conditions occur:
    // * 1) Move gets to the heading (angle)
    // * 2) Driver stops the opmode running.
    // *
    // * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
    // * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    // *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    // *                     If a relative angle is required, add/subtract from current heading.
    // *
    public void turnToHeading(double maxTurnSpeed, double heading) {

        resetHeading();
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while ((Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction

            moveRobot(0, +turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    //**
    // * Method to obtain & hold a heading for a finite amount of time
    // * Move will stop once the requested time has elapsed
    // * This function is useful for giving the robot a moment to stabilize it's heading between movements.
    // *
    // * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
    // * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    // *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    // *                     If a relative angle is required, add/subtract from current heading.
    // * @param holdTime     Length of time (in seconds) to hold the specified heading.
    //
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while ((holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    //
    // * This method uses a Proportional Controller to determine how much steering correction is required.
    // *
    // * @param desiredHeading   The desired absolute heading (relative to last heading reset)
    // * @param proportionalGain Gain factor applied to heading error to obtain turning power.
    // * @return Turning power needed to get to required heading.
    //
    private double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    //
    // * This method takes separate drive (fwd/rev) and turn (right/left) requests,
    // * combines them, and applies the appropriate speed commands to the left and right wheel motors.
    // *
    // * @param drive forward motor speed
    // * @param turn  clockwise turning motor speed.
    //
    private void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.


        leftSpeed = drive - turn;
        rightSpeed = drive + turn;


        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFront.setPower(leftSpeed);
        leftRear.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        rightRear.setPower(rightSpeed);
    }

    //
    // * This method takes separate drive (fwd/rev) and turn (right/left) requests,
    // * combines them, and applies the appropriate speed commands to the left and right wheel motors.
    // *
    // * @param drive forward motor speed
    // * @param turn  clockwise turning motor speed.
    //
    private void strafeRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFront.setPower(leftSpeed);
        //since we are straffing, directions are different
        leftRear.setPower(rightSpeed);

        rightFront.setPower(rightSpeed);
        //since we are straffing, directions are different
        rightRear.setPower(leftSpeed);
    }

    public void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d:%7d:%7d", leftFront.getCurrentPosition(), leftRear.getCurrentPosition(),
                    rightFront.getCurrentPosition(), rightRear.getCurrentPosition());
        } else {
            //telemetry.addData("Motion", "Turning");
        }

        telemetry.update();
    }

    public double getRawHeading() {
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public double getRawRelativeHeading()
    {
        return (getRawHeading()-headingOffset);

    }

    public double getAbsoluteHeading()
    {
        double angles   = getRawHeading();

        if  (angles < 0)
        {
            return (360 + ( angles));
        }
        return ( angles);
    }

    public double  getAbsoluteRelativeHeading()
    {
        double angles = getRawRelativeHeading();

        // Normalize the error to be within +/- 180 degrees
        while (angles > 180) angles -= 360;
        while (angles <= -180) angles += 360;

        return angles;
    }

}