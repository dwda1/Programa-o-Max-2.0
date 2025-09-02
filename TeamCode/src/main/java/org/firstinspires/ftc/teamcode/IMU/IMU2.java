package org.firstinspires.ftc.teamcode.IMU;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMU2 extends OpMode {
    TestBench bench = new TestBench();
    double heading;

    @Override
    public void init() {
        bench.init(hardwareMap);

    }

    @Override
    public void loop() {
        telemetry.addData("Heading: ", bench.getHeading(AngleUnit.RADIANS));
        heading = bench.getHeading(AngleUnit.DEGREES);

        if (heading > -0.5 && heading < 0.5 ) {
            bench.setMotor(0.0);
        }
        else if (heading > 0.5) {
            bench.setMotor(0.5);
        }
        else {
            bench.setMotor(-0.5);
        }
        telemetry.addData("power ", bench.motor.getPower() );
        telemetry.update();

    }
}
