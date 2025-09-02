package org.firstinspires.ftc.teamcode.IMU;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp

public class ImuPratice extends OpMode {
    TestBench bench = new TestBench();

    @Override
    public void init() {
        bench.init(hardwareMap);

    }

    @Override
    public void loop() {
        telemetry.addData("Heading", bench.getHeading(AngleUnit.RADIANS));

    }
}

/*
1. return the heading in radians
2. when youe heading is <0.5, stop a motor. If its less than -0.5,
make your motor positive and if  its greater than .5, make your motor negative.
 */
