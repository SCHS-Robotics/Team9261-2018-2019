package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "saucany", group = "SHOEZ")

public class WHY_YOU_NEED_A_GYROSCOPE_DOT_PNG extends LinearOpMode {
    @Override
    public void runOpMode(){
        BNO055IMU dab = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
        parameters.loggingEnabled = false;

        dab.initialize(parameters);
        while(!dab.isGyroCalibrated() && !isStarted() && !isStopRequested()) {
            sleep(50);
        }

        telemetry.addLine("PIZZA IS READY");
        telemetry.update();

        waitForStart();

        double kp = 0.1;
        double ki = 0.000000001;
        double kd = 0.001;

        double target = 45;

        double elapsedTime = 1;
        double errorSum = 0;

        double previousError = 0;
        double error = Integer.MAX_VALUE;

        while(opModeIsActive() && Math.abs(error) < 0.1) {

            double startTime = System.currentTimeMillis();
            Orientation angles = dab.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = target - angles.firstAngle;

            double p = kp*error;
            double i = ki*(error+errorSum)*elapsedTime;
            double d = kd*(error-previousError)/elapsedTime;

            double pid = p+i+d;

            errorSum += error;

            telemetry.addData("Roll",dab.getAngularOrientation().secondAngle);
            telemetry.addData("Pitch",dab.getAngularOrientation().thirdAngle);
            telemetry.addData("Yeet",dab.getAngularOrientation().firstAngle);

            telemetry.update();
            previousError = error;
            elapsedTime = System.currentTimeMillis()-startTime;
        }
    }
}
