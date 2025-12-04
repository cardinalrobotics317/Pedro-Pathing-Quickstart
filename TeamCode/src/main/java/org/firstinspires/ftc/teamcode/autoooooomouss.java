package org.firstinspires.ftc.teamcode;

import android.provider.SyncStateContract;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.util.Constants;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
//Import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class autoooooomouss extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }
    PathState pathState;

    private final Pose startPose = new Pose(40.27364185110664, 8.692152917505023, Math.toRadians(0));
    private final Pose shootPose = new Pose(36.016, 108.603, Math.toRadians(136));

    private PathChain driveStartPosShootPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (follower.isBusy()) {
                    telemetry.addLine("Done Path 1");
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState mewState) {
        pathState = mewState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        //follower = Constants.createFollower

        buildPaths();
        follower.setPose(startPose);

    }
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop (){
        follower.update();
        statePathUpdate();

    }
}
