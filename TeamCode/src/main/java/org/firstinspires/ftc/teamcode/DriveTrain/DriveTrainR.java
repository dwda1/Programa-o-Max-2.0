package org.firstinspires.ftc.teamcode.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrainR extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB;

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

    }

    private void initHardware() {
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");

    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x * 1.1;
        double turn = gamepad1.right_stick_x;

    }

}

//Rodrigo