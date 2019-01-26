package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="PIDTuner",group = "")
public class PIDTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
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

        DcMotor zero = hardwareMap.dcMotor.get("Ella-x");
        DcMotor one = hardwareMap.dcMotor.get("Cole-x");
        DcMotor two = hardwareMap.dcMotor.get("Ella-y");
        DcMotor three = hardwareMap.dcMotor.get("Cole-y");

        //MediaPlayer chezbob = MediaPlayer.create(hardwareMap.appContext, R.raw.chezbob);

        //DcMotor lift = hardwareMap.dcMotor.get("lifty");

        zero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zero.setDirection(DcMotor.Direction.FORWARD);
        one.setDirection(DcMotor.Direction.REVERSE);
        two.setDirection(DcMotor.Direction.FORWARD);
        three.setDirection(DcMotor.Direction.REVERSE);

        //lift.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        double kp = 0;
        double ki = 0;
        double kd = 0;

        double target = 45;

        double elapsedTime = 1;
        double errorSum = 0;

        double previousError = 0;
        double error;

        while(opModeIsActive()) {

            kp += -gamepad1.right_stick_y / (gamepad1.right_stick_y + 150000);
            kd += -gamepad1.left_stick_y / (gamepad1.left_stick_y + 150000);
            ki += -gamepad2.right_stick_y / (gamepad2.right_stick_y + 150000);

            double startTime = System.currentTimeMillis();
            Orientation angles = dab.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = target - angles.firstAngle;

            double p = kp*error;
            double i = ki*(error+errorSum)*elapsedTime;
            double d = kd*(error-previousError)/elapsedTime;

            double pid = p+i+d;

            zero.setPower(+pid);
            one.setPower(-pid);
            two.setPower(pid);
            three.setPower(-pid);

            errorSum += error;

            telemetry.addData("Roll",dab.getAngularOrientation().secondAngle);
            telemetry.addData("Pitch",dab.getAngularOrientation().thirdAngle);
            telemetry.addData("Yeet",dab.getAngularOrientation().firstAngle);

            telemetry.addData("kp",kp);
            telemetry.addData("ki",ki);
            telemetry.addData("kd",kd);

            telemetry.update();
            previousError = error;
            elapsedTime = System.currentTimeMillis()-startTime;
        }
    }
}
