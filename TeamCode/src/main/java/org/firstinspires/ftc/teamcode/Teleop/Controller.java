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

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

 */
@TeleOp(name = "DriverControl_Pressme;)", group = "Wesley")
//@Disabled  This way it will run on the robot
public class Controller extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();  //timer
//____________________________________________________________________________Motor Declarations_______________________________________________________________________________________________________________________________________________________
    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;

//______________________________________________________________________________variables_______________________________________________________________________________________________________________________________________________________________________

    private final int[] armLevelPosition = {0, 260, 650, 995};
    private final boolean isGrabbing = false;
    private int armLevel;
    private double speedMod;
    private double previousRunTime;
    private final double inputDelayInSeconds = .5;
    private boolean rumbleLevel = true;
    private final boolean gripperToggle = false;
    private double rotation = 0;
//______________________________________________________________________________Init Code_____________________________________________________________________________________________________________________________________________________________________________
//motors initlize
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");




        //Motor Encoders
        //Wheels
        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized balls");


    }
//_________________________________________________________________________Drive code___________________________________________________________________________________________________________________________________________
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
        runtime.reset();
        previousRunTime = getRuntime();


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        precisionControl();
        drivingControl();
}
       if (gamepad1.left_trigger > 0) {
            //removed || gamepad2.left_trigger > 0)
            speedMod = .25;
            gamepad1.rumble(100);
            gamepad2.rumble(100);
        } else if (gamepad1.right_trigger > 0) {
            //removed || gamepad2.right_trigger > 0
            speedMod = 0.5;
            gamepad1.rumble(.5, .5, 1000);
            gamepad2.rumble(.5, .5, 1000);

        } else {
            speedMod = 1;
//            gamepad1.stopRumble();
//            gamepad2.stopRumble();

        }
    }

    public void drivingControl() {
        //gets controller input
        double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);

        //make calculations based upon the input
        double robotAngle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        rotation += 1 * rightX;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        //change the power for each wheel
        wheelFL.setPower(-v1 * speedMod);
        wheelFR.setPower(-v2 * speedMod);
        wheelBL.setPower(v3 * speedMod);
        wheelBR.setPower(v4 * speedMod);
    }

    public void intakeControl() {
        // if (intakeSensor.getState()) {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        //gamepad2.rumble(1000);
        // }
        // else{
        if (gamepad2.left_bumper) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else if (gamepad2.right_bumper) {
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
    }

    private void susanControl() {
//            if (gamepad1.dpad_left || gamepad2.dpad_left) {
//                susanWheel.setVelocity(945);
//                telemetry.addData("Status", "dpad left");
//            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
//                susanWheel.setVelocity(-945);
//                telemetry.addData("Status", "bumper right");
//            } else {
//                susanWheel.setVelocity(0);
//                //susanWheel.setPower(0);  //disabled to stop conteractingg
//
//                if (gamepad1.left_bumper) {
//                    susanWheel.setVelocity(6000);
//                    telemetry.addData("Status", "dpad left");
//                } else if (gamepad1.right_bumper) {
//                    susanWheel.setVelocity(-6000);
//                    telemetry.addData("Status", "bumper right");
//                } else {
//                    susanWheel.setVelocity(0);


        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            susanWheel.setPower(susanWheel.getPower() + 0.007);
            telemetry.addData("Status", "dpad left");
        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            susanWheel.setPower(susanWheel.getPower() - 0.007);
            telemetry.addData("Status", "bumper right");
        } else {
            susanWheel.setPower(0);  //disabled to stop conteractingg


        }

    }


    private void forkliftControl() {

        if ((gamepad1.dpad_up || gamepad2.dpad_up) && (armLevel < armLevelPosition.length - 1) && (getRuntime() - previousRunTime >= inputDelayInSeconds)) {
            rumbleLevel = true;
            previousRunTime = getRuntime();
            armLevel++;
        }
        if ((gamepad1.dpad_down || gamepad2.dpad_down) && (armLevel > 0) && (getRuntime() - previousRunTime >= inputDelayInSeconds)) {
            rumbleLevel = true;
            previousRunTime = getRuntime();
            armLevel--;


        }

        //sets to driving level
        if (gamepad1.y || gamepad2.y) {
            armLevel = 1;
        }

        armSlide.setVelocity(1000);
        if (armLevel == 1) {
            armSlide.setVelocity(2000);
            //if statement to set speed only going down
        }

        if (getRuntime() - previousRunTime >= inputDelayInSeconds + .25 && rumbleLevel) {
            rumbleLevel = false;
        }
        armSlide.setTargetPosition(armLevelPosition[armLevel]);
        armSlide.setTargetPositionTolerance(armLevelPosition[armLevel]);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    /*
     * Code to run ONCE after the driver hits STOP
     */

    private void tapething() {


        updown.setPower(gamepad2.left_stick_y *.25); //may need to put - in front if directin is flipped
        leftright.setPower(gamepad2.right_stick_x *.15);
        Outin.setPower(-gamepad2.left_trigger + gamepad2.right_trigger); //change the plus and - around if direction is flipped





//        if (gamepad2.left_stick_y > .1) ;
//        updown.setPower(.5);
//
//
//        if (gamepad2.left_stick_x > .1) ;
//        leftright.setPower(.5);
//
//        Outin.setPower(-gamepad2.left_trigger + gamepad2.right_trigger);


//        if (gamepad2.left_stick_y < .5) {
//            updown.setPower(1);
//        } else
//            updown.setPower(0);
//
//
//        }


    }

    public static void wait(int ms) {
        try {
            Thread.sleep(ms); //core java delay command
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt(); //this exception is useful to remove the glitches and errors of the thread.sleep()
        }
    }

    //@Override
    void Stop() {
        armSlide.setTargetPosition(0);
    }

}