package org.firstinspires.ftc.robotcontroller.external;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;


/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.


Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

@TeleOp(name="LcTeleOp18", group= "TeleOp")
public class LcTeleOp18 extends OpMode {



    // motor controllers
    DcMotorController driveController;
    DcMotorController armController;

    //servo controllers
    ServoController clawController;

    //servos
    Servo topRight;
    Servo topLeft;
    Servo bottomRight;
    Servo bottomLeft;


    // motors
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorLift;

    // slow mode settings
    int slowModeType = 4;
    String displaySlowModeType = "Quarter Speed";
    double leftMod = 0.92;
    double rightMod = 1.0;

    /**
     * Constructor
     */

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */


        // motor controllers
        driveController = hardwareMap.dcMotorController.get("driveController");
        armController = hardwareMap.dcMotorController.get("armController");
        //servo controllers
        clawController=hardwareMap.servoController.get("clawController");

        //dcMotors
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLift = hardwareMap.dcMotor.get("motorLift");
        //servos
        topRight = hardwareMap.servo.get("topRight");
        topLeft = hardwareMap.servo.get("topLeft");
        bottomRight = hardwareMap.servo.get("bottomRight");
        bottomLeft = hardwareMap.servo.get("bottomLeft");
        //sensors

        //Set all powers to 0
        motorRight.setPower(0);
        motorLeft.setPower(0);
        motorLift.setPower(0);

        //servo init
        topRight.setPosition(0);
        topLeft.setPosition(1);
        bottomRight.setPosition(1);
        bottomLeft.setPosition(0);


    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void init_loop() {

    }

    public void loop() {

		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick and right stick (tank drive)
		 * and the slow mode via the xyab buttons
		 */
        // slow mode type
        if(gamepad1.a){
            slowModeType = 1;
        }
        if(gamepad1.b){
            slowModeType = 2;
        }
        if(gamepad1.y) {
            slowModeType = 4;
        }
        if(gamepad1.x){
            slowModeType = 8;
        }
        //Tank Drive
        float right = gamepad1.right_stick_y;
        float left = -gamepad1.left_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float) scaleInput(right)*(float)rightMod;
        left = (float) scaleInput(left)*(float)leftMod;

        //Set the values to the motors
        motorRight.setPower(right/slowModeType);
        motorLeft.setPower(left/slowModeType);

    /*
    * Gamepad 2
    *
    * Gamepad 2 lifts the arm and uses the grabbing claw.
    */

    //LIFT

      //
        if(gamepad2.left_stick_y>0 || gamepad2.left_stick_y<0){
            double vert=gamepad2.left_stick_y; // arm is left stick
            motorLift.setPower(vert);
        }
        else if(gamepad2.left_stick_y==0){
            motorLift.setPower(0);
        }

    //CLAW

      // right bumper opens claw, left closes it.

        double openVal = 0.4;

        if (gamepad2.left_bumper) {
            topRight.setPosition(0+openVal);
            topLeft.setPosition(1-openVal);
            bottomRight.setPosition(1-openVal);
            bottomLeft.setPosition(0+openVal);

        }

        //push blocks in (fully closed)
        if (gamepad2.right_bumper){
            topRight.setPosition(0);
            topLeft.setPosition(1);
            bottomRight.setPosition(1);
            bottomLeft.setPosition(0);
        }


    //convert slowModeType to displaySlowModeType
        switch (slowModeType){
            case 1:
                displaySlowModeType = "Normal Speed";
                break;
            case 2:
                displaySlowModeType = "Half Speed";
                break;
            case 4:
                displaySlowModeType = "Quarter Speed";
                break;
            case 8:
                displaySlowModeType = "Eighth Speed";
                break;
            default:
                displaySlowModeType = "Error!";
                break;


		/*
		 * Send TELEMETRY data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Drive Speed modifier", "val: " + displaySlowModeType);
        telemetry.addData("Text", "*****MOTORS*****");
        telemetry.addData("left tgt pwr", "pwr: " + "%.2f", left);
        telemetry.addData("right tgt pwr", "pwr: " + "%.2f", right);
        telemetry.addData("arm vertical", "pwr: "+ "%.2f", motorLift.getPower());
        telemetry.update();
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */

    @Override
    public void stop() {

    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */

    public double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


}

