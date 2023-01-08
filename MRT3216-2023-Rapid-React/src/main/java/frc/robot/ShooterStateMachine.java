package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Ball;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterStateMachine {
    private static ShooterStateMachine instance;
    private Ball ball1;
    private Ball ball2;
    private boolean ball1InChute;
    private ShooterSubsystem shooterSystem;
    private ColorSensorSubsystem colorSystem;
    private int ballsShot;
    private double ballShotFilterThreshold;

    private ShooterStateMachine() {
        this.shooterSystem = ShooterSubsystem.getInstance();
        this.colorSystem = ColorSensorSubsystem.getInstance();
        this.ballShotFilterThreshold = Constants.Shooter.Flywheel.ballShotfilterThreshold;
        this.ball1 = Ball.NONE;
        this.ball2 = Ball.NONE;
        this.ball1InChute = false;
        this.ballsShot = 0;

        new Trigger(
                () -> shooterSystem.getFilterValue() < this.ballShotFilterThreshold)
                        .whileActiveOnce(new InstantCommand(() -> ballShot()));

        new Trigger(() -> colorSystem.inRange())
                .whileActiveOnce(new InstantCommand(() -> ballIndexed(colorSystem.isAllianceBall())));
    }

    public void setLastBall(Ball ball) {
        if (ball1InChute) {
            ball2 = ball;

        } else {
            if (ball1 != ball && ball1 != Ball.NONE) {
                setBallInChute(true);
            } else {
                ball1 = ball;
            }
        }
    }

    public int ballsShot() {
        return this.ballsShot;
    }

    public void resetShot() {
        this.ballsShot = 0;
    }

    public void reverseEject() {
        ball1 = Ball.NONE;
        ball2 = Ball.NONE;
        setBallInChute(false);
    }

    // Used for ShuffleBoard
    public String getBall1String() {
        return ball1.toString();
    }

    // Used for ShuffleBoard
    public String getBall2String() {
        return ball2.toString();
    }

    // Used for ShuffleBoard
    public boolean getBallInChute() {
        return this.ball1InChute;
    }

    private void ballIndexed(boolean allianceBall) {
        ball1 = allianceBall ? Ball.ALLIANCE : Ball.OPPONENT;
    }

    private void ballShot() {
        ball1 = ball2;
        ball2 = Ball.NONE;
        setBallInChute(false);
        ballsShot++;
    }

    private void setBallInChute(boolean isBall1InChute) {
        this.ball1InChute = isBall1InChute;
    }

    public static ShooterStateMachine getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new ShooterStateMachine();
        }
        return instance;
    }
}