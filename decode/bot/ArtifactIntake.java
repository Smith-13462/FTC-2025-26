package org.firstinspires.ftc.teamcode.decode.bot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.common.DecodeConstants;

public class ArtifactIntake {
    public DcMotorEx intakeMotor = null;
    private boolean intakeMotorAvailable = true;
    private ElapsedTime timeSinceLastToggle;
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
            intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
            intakeMotor.setPower(0);

            intakeMotorAvailable = true;
        } catch (Exception e) {
            intakeMotorAvailable = false;
            telemetry.addData("Init problem with ", "intake motor");
           // telemetry.update();
        }

        timeSinceLastToggle =new ElapsedTime();
        timeSinceLastToggle.reset();
    }

    public void turnOnIntake() {
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setPower(1);
    }

    public void turnOffIntake() {
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setPower(0);
    }

    public void reverseIntake() {
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setPower(1);
    }

    public void toggleIntake() {

        if(timeSinceLastToggle.time() < 1) return;

        if(isIntakeActive()){
            turnOffIntake();
        } else {
            turnOnIntake();
        }

        timeSinceLastToggle.reset();
    }
    public boolean isIntakeActive(){
        if(intakeMotor.getPower() > 0){
            return true;
        } else {
            return false;
        }
    }
}