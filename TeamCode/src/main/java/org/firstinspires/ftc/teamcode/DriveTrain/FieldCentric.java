package org.firstinspires.ftc.teamcode.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FieldCentric extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB;
    private double speed = 0.75;

    @Override
    public void runOpMode(){
        initHardware();

        RMF.setDirection(DcMotorSimple.Direction.REVERSE);
        RMB.setDirection(DcMotorSimple.Direction.REVERSE);
        LMF.setDirection(DcMotorSimple.Direction.FORWARD);
        LMB.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
        }

    }

    private void initHardware() {
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");

    }

}
