package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto")
public class ExampleAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56.30, 9.60, Math.toRadians(90)); // Start Pose of our robot.

    private final Pose scorePose = new Pose(63.75, 31.00, Math.toRadians(90));

    private final Pose pickUpPose = new Pose(25.00, 34.50, Math.toRadians(180));

    private final Pose controlPick = new Pose(44.12, 26.64, Math.toRadians(180));

    private final Pose pickUp2Pose = new Pose(25.30, 60.60, Math.toRadians(180));

    private final Pose control2Pick = new Pose(62.0, 54.0, Math.toRadians(180));


    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,controlPick,pickUpPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickUpPose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpPose,controlPick,scorePose))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(),scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,control2Pick,pickUp2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickUp2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUp2Pose,control2Pick,scorePose))
                .setLinearHeadingInterpolation(pickUp2Pose.getHeading(),scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 5.0){
                    Superstructure.setCurrentWantedState(Superstructure.wantedState.INTAKE);
                    setPathState(3);
                }

                break;
            case 3:
                if (!follower.isBusy()){
                    follower.followPath(grabPickup1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.followPath(scorePickup1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                   setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        Superstructure.init(hardwareMap);

    }

    @Override
    public void loop() {
        Superstructure.read();
        Superstructure.periodic();
        Superstructure.write();
        Superstructure.setAngularVel(follower.getAngularVelocity());
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
