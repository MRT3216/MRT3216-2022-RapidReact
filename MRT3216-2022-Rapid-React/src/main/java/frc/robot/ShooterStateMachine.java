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
    
    private double ballShotFilterThreshold;

    private ShooterStateMachine() {
        this.shooterSystem = ShooterSubsystem.getInstance();
        this.colorSystem = ColorSensorSubsystem.getInstance();
        this.ballShotFilterThreshold = Constants.Shooter.Flywheel.ballShotfilterThreshold;
        ball1 = Ball.NONE;
        ball2 = Ball.NONE;
        ball1InChute = false;

        new Trigger(
                () -> shooterSystem.getFilterValue() < this.ballShotFilterThreshold)
                        .whileActiveOnce(new InstantCommand(() -> ballShot()));

        new Trigger(() -> colorSystem.inRange())
                .whileActiveOnce(new InstantCommand(() -> ballIndexed(colorSystem.isAllianceBall())));
    }

    public void ballShot() {
        ball1 = ball2;
        ball2 = Ball.NONE;
        setBallInChute(false);
        // System.out.println("Ball shot!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    public void ballIndexed(boolean allianceBall) {
        ball1 = allianceBall ? Ball.ALLIANCE : Ball.OPPONENT;
        // System.out.println("Ball indexed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    public void reverseEject() {
        ball1 = Ball.NONE;
        ball2 = Ball.NONE;
        setBallInChute(false);
    }

    public Ball getBall1() {
        return ball1;
    }

    public String getBall1String() {
        return ball1.toString();
    }

    public Ball getBall2() {
        return ball2;
    }

    public String getBall2String() {
        return ball2.toString();
    }

    public void setBall1(Ball ball) {
        this.ball1 = ball;
    }

    public void setBall2(Ball ball) {
        this.ball2 = ball;
    }

    public void setLastBall(Ball ball) {
        // System.out.println("Set Last Ball to: " + ball.toString() + "
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        if (ball1InChute) {
            ball2 = ball;
            // System.out.println("Set Ball 2 to: " + ball.toString() + "
            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        } else {
            if (ball1 != ball && ball1 != Ball.NONE) {
                // ball1Changed = true;
                setBallInChute(true);
            } else {
                ball1 = ball;
            }
        }
        // System.out.println("Set Ball 1 to: " + ball.toString() + "
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    public boolean getBallInChute() {
        return this.ball1InChute;
    }

    public void setBallInChute(boolean isBall1InChute) {
        // System.out.println("Set Ball in chute from " + this.ball1InChute + " to " +
        // isBall1InChute);
        this.ball1InChute = isBall1InChute;
    }

    public void setBallFilterThreshold(double ballShotFilterThreshold) {
        this.ballShotFilterThreshold = ballShotFilterThreshold;
    }

    public static ShooterStateMachine getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new ShooterStateMachine();
        }
        return instance;
    }
}