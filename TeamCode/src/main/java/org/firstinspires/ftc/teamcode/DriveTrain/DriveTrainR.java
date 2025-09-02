package org.firstinspires.ftc.teamcode.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DriveTrainR extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB;
    private double speed = 0.75;

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();
        while (opModeIsActive()) {
            drive();

        }

    }

    private void initHardware() {
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");

        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x * 1.1;
        double turn = gamepad1.right_stick_x;

        drive = applyDeadzone(drive);
        strafe = applyDeadzone(strafe);
        turn = applyDeadzone(turn);

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        double powerRMF = (drive - strafe - turn) / denominator;
        double powerLMF = (drive + strafe + turn) / denominator;
        double powerRMB = (drive + strafe - turn) / denominator;
        double powerLMB = (drive - strafe + turn) / denominator;

        RMF.setPower(powerRMF * speed);
        LMF.setPower(powerLMF * speed);
        RMB.setPower(powerRMB * speed);
        LMB.setPower(powerLMB * speed);

    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }

}

//Rodrigo