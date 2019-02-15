package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PleaseWork", group="banana")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor uno = hardwareMap.dcMotor.get("Ella-x");
        uno.setDirection(DcMotor.Direction.FORWARD);
        DcMotor dos = hardwareMap.dcMotor.get("Cole-x");
        dos.setDirection(DcMotor.Direction.FORWARD);
        DcMotor tres = hardwareMap.dcMotor.get("Ella-y");
        tres.setDirection(DcMotor.Direction.REVERSE);
        DcMotor cuatro = hardwareMap.dcMotor.get("Cole-y");
        cuatro.setDirection(DcMotor.Direction.REVERSE);

        TouchSensor cinco = hardwareMap.touchSensor.get("shoe");
        ColorSensor seis = hardwareMap.colorSensor.get("shirt");

        waitForStart();

        //! not true = false, not false = true
        //&&0000
        //||

        while (opModeIsActive()) {
            telemetry.addData("ANTI-THEFT ACTIVATED",cinco.isPressed());
            telemetry.addData( "The value of red is",seis.red());
            telemetry.update();
            if (!cinco.isPressed() && (seis.red() >= 40)) {
                uno.setPower(Range.clip(gamepad1.right_stick_y - gamepad1.right_stick_x, -1, 1));
                dos.setPower(Range.clip(gamepad1.right_stick_y - gamepad1.right_stick_x, -1, 1));
                tres.setPower(Range.clip(gamepad1.right_stick_y + gamepad1.right_stick_x, -1, 1));
                cuatro.setPower(Range.clip(gamepad1.right_stick_y + gamepad1.right_stick_x, -1, 1));

            }
            else {
                uno.setPower(0);
                dos.setPower(0);
                tres.setPower(0);
                cuatro.setPower(0);
            }

        }
    }
}
