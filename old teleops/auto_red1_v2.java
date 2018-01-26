package org.firstinspires.ftc.robotcontroller.external;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="auto_red1_v2", group= "Autonomous")
public class auto_red1_v2 extends LinearOpMode {

    // motor controllers
    DcMotorController driveController;
    DcMotorController armController;

    //servo controllers
    ServoController clawController;

    // motors
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorLift;

    //servos
    Servo topRight;
    Servo topLeft;
    Servo bottomRight;
    Servo bottomLeft;

    private ElapsedTime     runtime = new ElapsedTime();

    //set encoder stuff
    static final double     COUNTS_PER_WHEEL_REV_R    = 23 ;
    static final double     COUNTS_PER_WHEEL_REV_L    = 22 ;// our motor dist
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_R         = (COUNTS_PER_WHEEL_REV_R)/
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     COUNTS_PER_INCH_L        = (COUNTS_PER_WHEEL_REV_L)/
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.5;

    int balanceDist=14; //the distance required to get off of the balance stone
	
    @Override


    public void runOpMode() {

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

        //close the claw
        topRight.setPosition(1);
        topLeft.setPosition(0);
        bottomRight.setPosition(0);
        bottomLeft.setPosition(1);

		//swap right direction
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

		//reset encoders
		motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		//run encoders
		motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		telemetry.addData("Path0",  "Starting at %7d :%7d",
                          motorLeft.getCurrentPosition(),
                          motorRight.getCurrentPosition());
        telemetry.update();
		
		// Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*

        ====================================================================
                                DRIVING COMMANDS
        ====================================================================

        */

        //lift the block a bit
        motorLift.setPower(-0.5);
        sleep(800);
        motorLift.setPower(0);

        encoderDrive(DRIVE_SPEED, balanceDist+3,balanceDist+4,4.0,0.5); //drive off platform
        encoderDrive(0.2,-6,-6, 1.0,0.5); //back up and square ourselves on the stone
        encoderDrive(0.2, 22-balanceDist,22-balanceDist,3.0,1); //go back to turning point
        encoderDrive(TURN_SPEED, -4,4,3.0,1); //turn towards goal
        encoderDrive(DRIVE_SPEED, 14,14,4.0,0.5); //pull up to goal

        dropBlock(); //drop the block


        /*

        ====================================================================
                                         END
        ====================================================================

        */

        telemetry.addData("Path", "Complete");
        telemetry.update();
	}
		
	public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS, double wait){
            int leftTarget;
			int rightTarget;
			double leftMod = 0.92;
			double rightMod = 1.0;
				// Ensure that the opmode is still active
			if (opModeIsActive()) {

				// Determine new target position, and pass to motor controller
				leftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH_L);
				rightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH_R);

				leftTarget = (int)(leftTarget*leftMod);
				rightTarget = (int)(rightTarget*rightMod);

				motorLeft.setTargetPosition(leftTarget);
				motorRight.setTargetPosition(rightTarget);
	
				// Turn On RUN_TO_POSITION
				motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				// reset the timeout time and start motion.
				runtime.reset();
				motorLeft.setPower(Math.abs(speed)*leftMod);
				motorRight.setPower(Math.abs(speed)*rightMod);

				// keep looping while we are still active, and there is time left, and both motors are running.
				// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
				// its target position, the motion will stop.  This is "safer" in the event that the robot will
				// always end the motion as soon as possible.
				// However, if you require that BOTH motors have finished their moves before the robot continues
				// onto the next step, use (isBusy() || isBusy()) in the loop test.
				while (opModeIsActive() &&
					(runtime.seconds() < timeoutS) &&
					(motorLeft.isBusy() || motorRight.isBusy())) {

					// Display it for the driver.
					telemetry.addData("Path1",  "Running to %7d :%7d", leftTarget,  rightTarget);
					telemetry.addData("Path2",  "Running at %7d :%7d",
                                            motorLeft.getCurrentPosition(),
                                            motorRight.getCurrentPosition());
					telemetry.update();
				}

				// Stop all motion;
				motorLeft.setPower(0);
				motorRight.setPower(0);

				// Turn off RUN_TO_POSITION
				motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

				sleep((long)wait*1000);   // optional pause after each move
			}
	}

	public void dropBlock(){
	    //lower the block down a bit
	    motorLift.setPower(0.5);
	    sleep(500);
	    motorLift.setPower(0);
        sleep(500);
        //let go of the block
        topRight.setPosition(0.7);
        topLeft.setPosition(0.2);
        bottomRight.setPosition(0.2);
        bottomLeft.setPosition(0.7);
    }
}
