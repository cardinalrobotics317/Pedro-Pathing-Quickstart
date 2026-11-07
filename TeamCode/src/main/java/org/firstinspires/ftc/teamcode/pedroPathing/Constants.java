package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(16.2)
            .forwardZeroPowerAcceleration(-44.3)
            .lateralZeroPowerAcceleration(-60.0)

            // PRIMARY TRANSLATIONAL PID = ZERO
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.06, 0, 0.001, 0.05
            ))
            .translationalPIDFSwitch(0)

            // SECONDARY TRANSLATIONAL PID = ZERO
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0, 0, 0, 0
            ))

            // PRIMARY HEADING PID = ZERO
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.3, 0, .05, 0.05
            ))

            // SECONDARY HEADING PID = ZERO
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0, 0, 0, 0
            ))

            // PRIMARY DRIVE PIDF = ZERO
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0, 0, 0, 0, 0.5
            ))

            // SECONDARY DRIVE PIDF = ZERO
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0, 0, 0, 0, 0
            ))

            .drivePIDFSwitch(0)
            .centripetalScaling(0);

    // YOUR DRIVETRAIN CONSTANTS
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFrontDrive")
            .leftRearMotorName("leftBackDrive")
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(40.0)
            .yVelocity(23.0);

    // PINPOINT CONSTANTS
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.75)
            .strafePodX(-6.6)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .hardwareMapName("pinpoint");

    // PATH CONSTRAINTS (UNCHANGED)
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            100,
            1,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}