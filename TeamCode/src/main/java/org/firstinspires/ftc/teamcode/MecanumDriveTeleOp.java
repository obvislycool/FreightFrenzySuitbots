package org.firstinspires.ftc.teamcode;
/**
 * @Author Edward Feldman
 * @Version 1.1
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Drive", group="Iterative Opmode")
//@Disabled
public class MecanumDriveTeleOp extends OpMode {

    final static double DONT_DESTROY_MOTORS = 0.70;
    final static double DUCK_SPEED = 0.45;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lfDrive = null;
    private DcMotor rfDrive = null;
    private DcMotor lbDrive = null;
    private DcMotor rbDrive = null;
    private DcMotor duckDrive = null;

    private boolean turboModeOn = false;
    private boolean pressingA = false;

    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        lfDrive  = hardwareMap.get(DcMotor.class, "lf");
        rfDrive = hardwareMap.get(DcMotor.class, "rf");
        lbDrive = hardwareMap.get(DcMotor.class, "lb");
        rbDrive = hardwareMap.get(DcMotor.class, "rb");
        duckDrive = hardwareMap.get(DcMotor.class, "spinnyDDuck");

        lfDrive.setDirection(DcMotor.Direction.FORWARD);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);
        lbDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rbDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        duckDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        double lfPower;
        double rfPower;
        double lbPower;
        double rbPower;
        double duckPower;

        double y1 = gamepad1.left_stick_y; //temporary bug
        double x1 = gamepad1.left_stick_x; //temporary bug
        double x2 = gamepad1.right_stick_x;

        //telemetry.addData("X1",x1);  bugfixing

        if(gamepad1.dpad_left){
            duckPower = -DUCK_SPEED;
        }else if(gamepad1.dpad_right){
            duckPower = DUCK_SPEED;
        }else{
            duckPower = 0;
        }


        lfPower = (y1+x1-x2);
        rfPower = (y1+x1+x2);
        lbPower = (y1-x1-x2);
        rbPower = (y1-x1+x2);

        double maxPower = Math.max(lfPower,Math.max(rfPower,Math.max(lbPower,Math.max(rbPower,1))));


        lfPower /= maxPower;
        rfPower /= maxPower;
        lbPower /= maxPower;
        rbPower /= maxPower;

        if(gamepad1.a && !pressingA){
            turboModeOn = !turboModeOn;
            pressingA = true;
        }
        if(!gamepad1.a){
            pressingA = false;
        }

        /*
        if(!(gamepad1.left_bumper || gamepad1.right_bumper)){

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            lfPower = (-y-x)*DONT_DESTROY_MOTORS;
            rfPower = (-y-x)*DONT_DESTROY_MOTORS;
            lbPower = (-y+x)*DONT_DESTROY_MOTORS;
            rbPower = (-y+x)*DONT_DESTROY_MOTORS;

        }
        else if (gamepad1.left_bumper){

            lfPower = -DONT_DESTROY_MOTORS;
            rfPower =  DONT_DESTROY_MOTORS;
            lbPower = -DONT_DESTROY_MOTORS;
            rbPower =  DONT_DESTROY_MOTORS;

        }
        else if (gamepad1.right_bumper){

            lfPower =  DONT_DESTROY_MOTORS;
            rfPower = -DONT_DESTROY_MOTORS;
            lbPower =  DONT_DESTROY_MOTORS;
            rbPower = -DONT_DESTROY_MOTORS;


        }
        else{
            telemetry.addLine("ERROR, NO MOTOR INPUT!");
        }
        */

        lfDrive.setPower((turboModeOn) ? lfPower : lfPower*DONT_DESTROY_MOTORS);
        rfDrive.setPower((turboModeOn) ? rfPower : rfPower*DONT_DESTROY_MOTORS);
        lbDrive.setPower((turboModeOn) ? lbPower : lbPower*DONT_DESTROY_MOTORS);
        rbDrive.setPower((turboModeOn) ? rbPower : rbPower*DONT_DESTROY_MOTORS);
        duckDrive.setPower(duckPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f), right front (%.2f), left back (%.2f), right back (%.2f)", lfPower, rfPower, lbPower, rbPower);
    }

    @Override
    public void stop() {
    }
}
