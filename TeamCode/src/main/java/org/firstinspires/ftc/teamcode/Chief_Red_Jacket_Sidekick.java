package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Cole helped me", group = "group")
public class Chief_Red_Jacket_Sidekick extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor one = hardwareMap.dcMotor.get("Cole-y");
        one.setDirection(DcMotor.Direction.REVERSE);
        DcMotor dos = hardwareMap.dcMotor.get("Tristan-y");
        dos.setDirection(DcMotor.Direction.FORWARD);
        DcMotor tres = hardwareMap.dcMotor.get("Cole-x");
        one.setDirection(DcMotor.Direction.FORWARD);
        DcMotor cuatro = hardwareMap.dcMotor.get("Tristan-x");
        dos.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        one.setPower(.5);
        dos.setPower(.5);
        tres.setPower(.5);
        cuatro.setPower(.5);
        sleep(9000);
        one.setPower(0);
        dos.setPower(0);
        tres.setPower(0);
        cuatro.setPower(0);
        

    }
}
