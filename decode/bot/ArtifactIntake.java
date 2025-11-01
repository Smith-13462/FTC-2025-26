package org.firstinspires.ftc.teamcode.decode.bot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

public class ArtifactIntake {
    static final double INTAKE_MOTOR_MAX_SPEED = 1.0;
    static final double INTAKE_MOTOR_TICKS_PER_ROTATION = 28 ;
    static final double INTAKE_MOTOR_RPM = 6000 ;
    static final double INTAKE_MOTOR_MAX_TICKS_PER_SECOND =
            (INTAKE_MOTOR_TICKS_PER_ROTATION * INTAKE_MOTOR_RPM) / 60;
    public DcMotorEx intakeMotor = null;
    private boolean intakeMotorAvailable = true;
    private double currentPower = 0, currentTicksPerSecond = 0;
    private Telemetry telemetry;
    ArtifactIntake(HardwareMap hardwareMap, DecodeConstants.OP_MODE op_mode, Telemetry telemetry) {
        this.telemetry = telemetry;
        InitializeIntake(hardwareMap, op_mode);
    }
    public void InitializeIntake(HardwareMap hardwareMap, DecodeConstants.OP_MODE op_mode) {

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakemotor");
            intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
            intakeMotor.setPower(0);

            intakeMotorAvailable = true;
        } catch (Exception e) {
            intakeMotorAvailable = false;
            telemetry.addData("Init problem with ", "intake motor");
           // telemetry.update();
        }
    }

    public void adjustIntakePower(double speedAdjustmentFactor) {
        if (speedAdjustmentFactor > 1) {
            speedAdjustmentFactor = 1;
        } else if (speedAdjustmentFactor < 0) {
            speedAdjustmentFactor = 0;
        }

        double adjustedPower = INTAKE_MOTOR_MAX_SPEED * speedAdjustmentFactor;

        if (intakeMotorAvailable) {
            intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setPower(adjustedPower);

            this.currentPower = adjustedPower;
        }
    }

    private void turnOffIntakePower() {
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
    }

    public void autoPickup(DecodeConstants.FieldPosition position){
        ElapsedTime holdClock = new ElapsedTime();
        while(holdClock.time() < 3){};

    }
}