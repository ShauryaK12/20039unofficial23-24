/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove ~or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Main Driver", group="Isaiah")
//@Disabled  This way it will run on the robot
public class DriveMain extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();  //timer

    /*
    Declare motors to type DcMotorEx
    Documentation:
    https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
     */
    private Rev2mDistanceSensor distanceR;
    private Rev2mDistanceSensor distanceB;
    private Rev2mDistanceSensor distanceL;

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private Servo wheel;
    private Servo stinger;

    private DcMotor motorArm;
    private double maxTicsPerSec = 3000;
    private int targetPosition;
    private DigitalChannel armMagnet;
    private int armLevel1st;
    private int armLevelRide;
    private int armLevelCanTilt;
    private int armLevel2nd;
    private int armLevel3rd;
    private int armLevel4;
    private boolean armZeroOverride;
    private String buttonPressed;

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MIN_POS = 0.0;     // Maximum rotational position
    static final double MAX_POS = 1.0;     // Minimum rotational position\

    double ppPos = 0.0; // Start at bottom position
    boolean rampUp = true;

    private int level = 1; //1 = ground, 2 = level 2, 3 = level 3
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");
        //arm = hardwareMap.get(DcMotorEx.class, "arm");

        wheel = hardwareMap.get(Servo.class,"wheel");
        stinger = hardwareMap.get(Servo.class,"stinger");

        motorArm = hardwareMap.get(DcMotor.class, "arm");
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
//        this.armZeroOverride = false;
        this.armLevel1st = 10;
//        this.armLevelRide = 500;
//        this.armLevelCanTilt = 1340;
        this.armLevel2nd = 1650;
        this.armLevel3rd = 2200;
        this.armLevel4 = 2200;
        this.targetPosition = 0;
        this.buttonPressed = "";



       /*
           Set up motors so they run without the encoders
           This way they run freely.  They won't go to a specific position or count the number of rotations
           It will now run with range from -1.0 to 1.0
           See Documentation for other encoder modes
           https://docs.revrobotics.com/rev-control-system/programming/using-encoder-feedback
        */
        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);
       // arm.setDirection(DcMotorSimple.Direction.FORWARD);
        ppPos = 1.0;


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //mecanum wheel code
        //https://ftccats.github.io/software/ProgrammingMecanumWheels.html

        //Get game controller input
        double r = Math.hypot((gamepad1.left_stick_x * -1), gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, (gamepad1.left_stick_x * -1)) - Math.PI / 4;
        double rightX = (gamepad1.right_stick_x * -1);

        //make calculations based upon the input
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        double lazyPower = 0.0;
        double spinnerPower = 0.0;
        double armPower = 0.0;


        //Lazy Susan
        if (gamepad1.y){
            lazyPower = 0.24;
            telemetry.addData("power ", "%.1f power", lazyPower);
            telemetry.update();
        }
        else if (gamepad1.x) {
            lazyPower = -0.24;
            telemetry.addData("power ", "%.1f power", lazyPower);
            telemetry.update();
        } else {
            lazyPower = 0.0;
            telemetry.addData("power ", "%.1f power", lazyPower);

        }

        if (gamepad2.dpad_left) {
            // Keep stepping up until we hit the max value.
            //telemetry.addData("position of servo", "%.1f", position);
            wheel.setPosition(wheel.getPosition()+0.01);
        }
        else if (gamepad2.dpad_right) {
            // Keep stepping down until we hit the min value.
            //  telemetry.addData("position of servo", "%.1f", position);
            wheel.setPosition(wheel.getPosition()-0.01);
        }

        if (gamepad2.dpad_up) {
            // Keep stepping up until we hit the max value.
            //telemetry.addData("position of servo", "%.1f", position);
            ppPos += INCREMENT ;
            stinger.setPosition(ppPos);
            if (ppPos >= MAX_POS) {
                ppPos = MAX_POS;
            }
        }
        else if (gamepad2.dpad_down) {
            // Keep stepping down until we hit the min value.
            //  telemetry.addData("position of servo", "%.1f", position);
            ppPos -= INCREMENT ;
            stinger.setPosition(ppPos);
            if (ppPos <= MIN_POS) {
                ppPos = MIN_POS;
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            //bucket.setPosition(position);
            telemetry.addData("position of servo", "%.1f", ppPos);
        }


        //change the power for each wheel

        wheelFL.setPower(gamepad1.x ? v1 * 0.5 : v1);
        wheelFR.setPower(gamepad1.x ? v2 * 0.5 : v2);
        wheelBL.setPower(gamepad1.x ? v3 * 0.5 : v3);
        wheelBR.setPower(gamepad1.x ? v4 * 0.5 : v4);
       //   ` arm.setPower(armPower);


        /*

        PLAYER 2 GAMEPAD


         */
        //get input
        if (gamepad2.x){ //2nd level
            targetPosition = armLevel3rd;
            buttonPressed = "X";
        }
        else if (gamepad2.y){  //3rd level
            targetPosition = armLevel4;
            buttonPressed = "Y";
        }
        else if (gamepad2.a){  //on floor
            targetPosition = armLevel1st;
            buttonPressed = "A";
        }
        else if (gamepad2.b){  //riding
            targetPosition = armLevel2nd;
            buttonPressed = "B";
        }
        else {
            buttonPressed = "";
        }

//        if



            /*
                Keep the target position >= 0
                Unless dpad_down is pressed
             */


            /*
                check if magnet is touching sensor
                True = NOT touching sensor
             */



        //set motor
        motorArm.setTargetPosition(targetPosition);
        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArm.setPower(maxTicsPerSec);

        telemetry.addData("Arm Button: ", buttonPressed);
        telemetry.addData("Can turn servo: ", motorArm.getCurrentPosition() >= armLevelCanTilt);
        telemetry.addData("Arm targetPosition: ", targetPosition);
        telemetry.addData("Arm currentPosition: ", motorArm.getCurrentPosition());
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public boolean distance(int cm) //cm = how far away to check
    {
        return true;
    }
}


