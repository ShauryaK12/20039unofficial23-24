package org.firstinspires.ftc.teamcode.markdrivesthehonda;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

@TeleOp(name = "Marko Drive", group = "honda")
public class MarkOp extends OpMode
{
    private ArrayList<String> activeTelemetry = new ArrayList<>();
    private DcMotorEx fLeft, fRight, bLeft, bRight;

    @Override
    public void init()
    {

        activeTelemetry.clear();

        try
        {
            // Load from hardware map
            fLeft = hardwareMap.get(DcMotorEx.class, HondaValues.hFrontLeftWheel.getValue());
            fRight = hardwareMap.get(DcMotorEx.class, HondaValues.hFrontRightWheel.getValue());
            bLeft = hardwareMap.get(DcMotorEx.class, HondaValues.hBackLeftWheel.getValue());
            bRight = hardwareMap.get(DcMotorEx.class, HondaValues.hBackRightWheel.getValue());

            // Set mode
            fLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            fRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            // Set direction
            fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            fRight.setDirection(DcMotorSimple.Direction.REVERSE);
            bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        }

        catch (Exception e)
        {
            TeleLog(HondaValues.tStatus, HondaValues.mFailed);
            TeleLog(HondaValues.tMotors,  HondaValues.mFailed);

            telemetry.update();

            throw e; // e again to throw the error again to see on tablet
        }

        // Tell the driver that initialization is complete.
        TeleLog(HondaValues.tStatus, HondaValues.mComplete);
        TeleLog(HondaValues.tMotors, HondaValues.mComplete);

        telemetry.update();

    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {

    }

    @Override
    public void loop()
    {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        fLeft.setPower(v1 * (0.5 + gamepad1.right_trigger));
        fRight.setPower(v2 * -(0.5 + gamepad1.right_trigger));
        bLeft.setPower(v3 * (0.5 + gamepad1.right_trigger));
        bRight.setPower(v4 * -(0.5 + gamepad1.right_trigger));


        // Thinking that GP1 is robot driver
        if(gamepad1.dpad_left) setPowerForMotors(-.5, .5, .5, -.5);
        else if(gamepad1.dpad_right) setPowerForMotors(.5, -.5, -.5, .5);
        else if(gamepad1.dpad_up) setPowerForMotors(1, 1, .5, .5);
        else if(gamepad1.dpad_down) setPowerForMotors(.5, .5, 1, 1);

    }

    @Override
    public void stop()
    {
        for (String str : activeTelemetry) telemetry.addData(str, HondaValues.mStopped.getValue());
        activeTelemetry.clear();
    }

    //region Utils

    // get"Xbox input"or"PS input"  this is how I named them
    // yes I understand we use PS controllers but just whenever
    // we do use xbox controllers. BOOM we have support for them
    protected static boolean getYorTriangle(Gamepad pad) { return (pad.y || pad.triangle);}
    protected static boolean getXorSquare  (Gamepad pad) { return (pad.x || pad.square);}
    protected static boolean getBorCircle  (Gamepad pad) { return (pad.b || pad.circle);}
    protected static boolean getAorCross   (Gamepad pad) { return (pad.a || pad.cross);}

    // When I was making this before I created this method everything
    // looked like .getValue() and it was annoying so this method I create.
    // still have to call manually telemetry.update()
    protected void TeleLog(HondaValues title, HondaValues message)
    {
        telemetry.addData(title.getValue(), message.getValue());

        for (String str : activeTelemetry) if(!str.equals(title.getValue())) activeTelemetry.add(title.getValue());
    }

    protected void setPowerForMotors(double fLeft, double fRight, double bLeft, double bRight)
    {
        this.fLeft.setPower(fLeft);
        this.fRight.setPower(fRight);
        this.bLeft.setPower(bLeft);
        this.bRight.setPower(bRight);
    }

    //endregion
}
