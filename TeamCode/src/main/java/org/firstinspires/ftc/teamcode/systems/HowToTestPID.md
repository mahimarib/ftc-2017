# How to Test PID Coefficients

Before you start you need to get familiar with what a PID controller
is here are some helpful links:

- My [presentation](https://docs.google.com/presentation/d/1uASkZ81vbdF8XrGbkZBF2osjTR3Mm9rGJ_aqqrXviRw/edit?usp=sharing)
  on motion profiling, you just need to look at page 9.
- This youtube [video](https://www.youtube.com/watch?v=pTuPhJ0DJB8).

## Brief Explanation of coefficients

- **Proportion** – The further you are away from the target, the faster the it will
                   output.
- **Integral** – Lets say you are trying to turn to 90º and you're at 88º and the
                 proportion output is at 0.002 (I'm just making a rough guess)
                 and that output and the weight of the robot will make it
                 impossible to finish the turn. The integral value basically
                 adds up the error over time so if you're stuck for too long
                 it will increase the value of the output until the target
                 is reached.
- **Derivatives** – The greater the rate of change, the more it dampens the
                    output, it is used to stabilize the output, and prevents
                    overshooting.

![pid gif](https://upload.wikimedia.org/wikipedia/commons/3/33/PID_Compensation_Animated.gif)

From the animation you can see that if you have a high `kP` it will overshoot
oscillate, as well as `kI`. You can use `kD` to dampen down the oscillation
and overshooting.

## Effects of Increasing a Parameter Independently

Here is a handy-dandy table:

Parameter | Rise time | Overshoot | Settling time | Steady-State error | Stability
:---: | :---: | :---: | :---: | :---: | :---:
`kP` | Decrease | Increase | Small change | Decrease | Degrade
`kI` | Decrease | Increase | Increase | Eliminate | Degrade
`kD` | Minor change | Decrease | Decrease | No effect in theory | Improve if `kD` is small

I have already written the code so don't worry, you just need to test the code
using different coefficient values. So you'll just be editing 3 lines of code.

## Picking Right Values

In the file [`MecanumDriveSystem.java`](MecanumDriveSystem.java#L114) lines
114 - 116 you have 3 PID coefficients:

```java
    double kP = 0.03;
    double kI = 0;
    double kD = 0.018;
```

Those values just happen to be what I used for my testing robot, you're going
to have to come up with your own values.

You might already know that a motor takes in values from `-1.0` to `1.0`,
and anything higher will just set the motor to `1.0` and anything lower
will set it to `-1.0`. So keep that in mind while looking at the code
snippet below:

```java
    double error = getAngleError(targetAngle);

    integral += error;
    double derivative = error - prevError;

    double kP_output = kP * error;
    double kI_output = kI * integral;
    double kD_output = kD * derivative;

    double output = Range.clip(
        kP_output + kI_output - kD_output, -maxSpeed, maxSpeed);
```

If you leave the coefficients as 0 it will not use it, for example if you
just want to use `kP` and `kD`, you will leave `kI` as 0. It will be called
KD controller.

You can see that the proportional output is `kP * error`, and the error
is just `targetAngle - currentAngle`. You have to keep in mind of what will be
the maximum turn angle. It would be just -180º and 180º. There would be no
reason to turn 270º since turning -90º will result the same orientation.

You have to keep in mind the magnitude of coefficients based on the numbers you
are working with, by magnitude I don't mean the 3 in `0.03`. If you write
0.03 in scientific notation it would be 3 * 10^(-2), and by magnitude I mean
how big the power is. If you really wanted to you can write the values in
scientific notation in code:

```java
    double kP = 3E-2;
    double kI = 0;
    double kD = 1.8E-2;
```

If you are going to turn 180º from 0º, the error would be 180. If `kP = 0.03`,
then `kP_output = 0.03 * 180 = 5.4`. You might be wondering that *"That's way
bigger than `1.0`"*, but if you make `kP = 0.003` then `kP_output = 0.54`, and
that is a small amount, it will just be going half the speed (unless that is
what you want), and the output will just keep getting smaller and smaller
until it reaches 0, and it will reach the target really slow. You would want a
higher value to have a quick autonomous. Just play around with the values.

Once you are done tuning you can run the code, it should be under the
autonomous tab and the name is turn test. It is set to turn 90º, you can change
the angle here: [`TurnTest.java`](../commands/testcommands/TurnTest.java)