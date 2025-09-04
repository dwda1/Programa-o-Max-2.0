package org.firstinspires.ftc.teamcode.DriveTrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentric extends LinearOpMode {

    private DcMotor RMF, RMB, LMF, LMB;
    private IMU imu;
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
            imu();
            drive();

            if (gamepad1.options) {
                imu.resetYaw();
            }

            telemetry.addData("IMU", imu.getRobotYawPitchRollAngles());
            telemetry.update();
        }

    }

    private void initHardware() {
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");

        imu = hardwareMap.get(IMU.class, "imu");

    }

    public void imu() {

        RevHubOrientationOnRobot RobotOrientaion = new RevHubOrientationOnRobot(
          RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
          RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        imu.initialize(new IMU.Parameters(RobotOrientaion));

    }

    public void drive() {
        double drive = - gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double driveY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
        double strafeX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);

        strafeX = strafeX * 1.1;

        driveY = apllyDeadzone(driveY);
        strafeX = apllyDeadzone(strafeX);
        turn = apllyDeadzone(turn);

        double denominator = Math.max(Math.abs(driveY) + Math.abs(strafeX) + Math.abs(turn), 1);
        double powerRMF = (driveY - strafeX - turn) / denominator;
        double powerRMB = (driveY + strafeX - turn) / denominator;
        double powerLMF = (driveY + strafeX + turn) / denominator;
        double powerLMB = (driveY - strafeX + turn) / denominator;

        RMF.setPower(powerRMF * speed);
        RMB.setPower(powerRMB * speed);
        LMF.setPower(powerLMF * speed);
        LMB.setPower(powerLMB * speed);

    }

    private double apllyDeadzone(double value) {
        return Math.abs(value) > 0.05 ? 0 : value;
    }

}

//Rotxz