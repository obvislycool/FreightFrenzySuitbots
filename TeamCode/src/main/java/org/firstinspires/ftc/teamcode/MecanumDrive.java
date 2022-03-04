package org.firstinspires.ftc.teamcode;
/**
 * @Author Edward Feldman
 * @Version 1.1
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum-Drive", group="Iterative Opmode")
//@Disabled
public class MecanumDrive extends OpMode {

    final static double DONT_DESTROY_MOTORS = 0.80;
    final static double DUCK_SPEED = 0.55;
    //final static double HARVEST_SPEED = 0.6;

    private ElapsedTime runtime = new ElapsedTime();
    HardwareDrivetrain robot   = new HardwareDrivetrain();

    private DcMotor lfDrive = null;
    private DcMotor rfDrive = null;
    private DcMotor lbDrive = null;
    private DcMotor rbDrive = null;
    private DcMotor duckDrive = null;
    private DcMotor slideMotor = null;
    private DcMotor harvestMotor = null;
    private Servo dumpServo = null;
    //private AnalogInput limitSensor = null;

    private boolean turboModeOn = false;
    private boolean pressingA = false;
    //private boolean pressingA2 = false;
    //private boolean dumperDown = false;

    private int slidePosition = 0; //0 is start, 1 is first, 2 is second, 3 is third

    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);

        lfDrive  = hardwareMap.get(DcMotor.class, "lf");
        rfDrive = hardwareMap.get(DcMotor.class, "rf");
        lbDrive = hardwareMap.get(DcMotor.class, "lb");
        rbDrive = hardwareMap.get(DcMotor.class, "rb");
        duckDrive = hardwareMap.get(DcMotor.class, "spinnyDDuck");
        slideMotor = hardwareMap.get(DcMotor.class, "slidemotor");
        harvestMotor = hardwareMap.get(DcMotor.class, "harvestmotor");

        //limitSensor = hardwareMap.get(AnalogInput.class, "limitsensor");

        dumpServo = hardwareMap.get(Servo.class, "dumpservo");

        lfDrive.setDirection(DcMotor.Direction.FORWARD);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);
        lbDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rbDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        duckDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        harvestMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.harvestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.harvestMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        duckDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Stops after input stops
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();


        //robot.slideMotor.setPower(-0.1);
        //while(buttonNotPressed && opModeActive && ticks < 350){
        //    sleep(0)
        //}
        //robot.slideMotor.setPower(0);
        //robot.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void loop() {


        double lfPower;
        double rfPower;
        double lbPower;
        double rbPower;
        double duckPower;
        double slidePower;
        double harvestPower;

        double dumpPosition;

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

        harvestPower = gamepad2.right_stick_y * -1;

        slidePower = gamepad2.left_stick_y * -.8;
        if(slidePower<0.1){
            slidePower *= 0.5;
            if(robot.limitSensor.isPressed()){
                slidePower=0;
                telemetry.addData("Status", "Slide Reached Limit");    //
                telemetry.update();
            }
        }



        if(gamepad2.a){
            dumpPosition = 0;
        }
        else{
            dumpPosition = .45;
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
        slideMotor.setPower(slidePower);
        harvestMotor.setPower(harvestPower);

        dumpServo.setPosition(dumpPosition);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f), right front (%.2f), left back (%.2f), right back (%.2f)", lfPower, rfPower, lbPower, rbPower);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLFTarget;
        int newRFTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLFTarget = robot.lfDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRFTarget = robot.rfDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        robot.lfDrive.setTargetPosition(newLFTarget);
        robot.rfDrive.setTargetPosition(newRFTarget);
        robot.lbDrive.setTargetPosition(newLFTarget);
        robot.rbDrive.setTargetPosition(newRFTarget);

        // Turn On RUN_TO_POSITION
        robot.lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.lfDrive.setPower(Math.abs(speed));
        robot.rfDrive.setPower(Math.abs(speed));
        robot.lbDrive.setPower(Math.abs(speed));
        robot.rbDrive.setPower(Math.abs(speed));
    }

    @Override
    public void stop() {
    }
}
