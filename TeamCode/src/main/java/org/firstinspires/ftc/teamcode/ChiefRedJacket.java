package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="test", group="banana")
public class ChiefRedJacket extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor m1 = hardwareMap.dcMotor.get("hi");
        m1.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        m1.setPower(0.7);
        sleep(9000);
        m1.setPower(0);
    }
}
