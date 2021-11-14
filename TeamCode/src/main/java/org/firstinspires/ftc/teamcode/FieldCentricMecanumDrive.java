package org.firstinspires.ftc.teamcode;
/**
 * @Author Edward Feldman
 * @Version 1.1
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Field-Centric Drive", group="Iterative Opmode")
//@Disabled
public class FieldCentricMecanumDrive extends OpMode {

    final static double DONT_DESTROY_MOTORS = 0.70;
    final static double DUCK_SPEED = 0.4;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lfDrive = null;
    private DcMotor rfDrive = null;
    private DcMotor lbDrive = null;
    private DcMotor rbDrive = null;
    private DcMotor duckDrive = null;

    BNO055IMU imu;
    Orientation angles;

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;
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

        double targetAngle = Math.atan2(y1, x1);
        double targetMagnitude = Math.sqrt(x1 * x1 + y1 * y1);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double getAngle = angles.firstAngle;

        double perspectiveAngle = targetAngle - getAngle;

        double perspectiveX1 = Math.cos(perspectiveAngle);
        double perspectiveY1 = Math.sin(perspectiveAngle);

        perspectiveX1 *= targetMagnitude;
        perspectiveY1 *= targetMagnitude;



        lfPower = (perspectiveY1+perspectiveX1-x2);
        rfPower = (perspectiveY1+perspectiveX1+x2);
        lbPower = (perspectiveY1-perspectiveX1-x2);
        rbPower = (perspectiveY1-perspectiveX1+x2);

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
