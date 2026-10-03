# Screw Divers Teamcode

Robot code for **Screw Divers** V0 robot, for the **Biobuzz** FTC season.

---

## Tools used

* **FTC SDK**, for main robot control.
* **Pedro Pathing**, for localization and path following.
* **Ivy**, for command scheduling and requirements and cleaner autos.
* **Panels**, for live telemetry and tuning from a browser for PIDs and stuff.
* **PsiKit**, for AdvantageKit-style logging and match reviewing.

---

## Repository layout

All robot code lives under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

```
teamcode/
├── Robot.java                 Builds every subsystem and scheudles everything
├── OpModes/
│   ├── CommandOpMode.java     Base class, alliance selector during init, command loop, telemetry, logging
│   ├── teleop/                
│   ├── autonomous/            
│   └── utility/               
├── subsystems/
│   ├── Intake.java            
│   ├── Shooter.java           
│   └── Turret.java            
├── utils/
│   ├── Scorer.java            combines the Shooter and Turret subsystems to make a full shoot commnad
│   ├── Alliance.java
│   ├── LazyMotor.java         Motor wrapper, writes only on change, cached current/velocity for looptimes
│   ├── LazyServo.java         Servo wrapper, writes only on change, cached position for looptimes
│   ├── RobotLogger.java       PsiKit logging for each subsystem
│   └── math/
│       ├── LookUpTable.java   LookUpTable code by demacia ❤️, used for shooter distance -> RPM/hood pos
│       ├── SOTM.java          calculations for SOTM
│       ├── LowPassFilter.java LowPassFilter, takes alpha value in constructor
│       └── MedianFilter.java  MedianFilter, takes window size in constructor
└── pedro/
    ├── Constants.java         Follower, drivetrain and localizer setup (to be filled in)
    ├── Tuning.java            
    └── procedures/            
```

---

## How to actually use it

- **OpModes only schedule and cancel commands.** Subsystems expose `Command`-returning methods such as `intakeCommand()`, `stopCommand()`, `spinUpCommand()` and `shootCommand()`. Their actual methods are private so everything stays clean and nice.
- **Every subsystem has a `periodic()` command.** It is `infinite`, `.requiring(...)` its hardware, and uses `InterruptedBehavior.SUSPEND`. Ivy's default is `END`, so without `SUSPEND` the control loop would die the first time a manual command interrupted it. These are scheduled once in `Robot.start()`.
- **Lazy hardware wrappers.** `LazyMotor` and `LazyServo` skip writes that would not change anything noticeably, which keeps hub traffic and loop times down.
- **Everything that needs tuning should be `public static` and `@Configurable`**, so it can be changed live from Panels.

---

## Logging and telemetry

- **Logging uses PsiKit.** Add `@PsiKitAutoLog` to the start of each OpMode and use `logger.set{subsystem}Logging(true)` for the subsystems you want to log.
- **Log files** are written on the Robot Controller under `/sdcard/FIRST/PsiKit/`.
- **Panels** shows telemetry and exposes every `@Configurable` value for live tuning.

---

## TODO

- [ ] Tune the follower using pedro.
- [ ] Autonomous OpModes (none exist yet).
- [ ] Transfer subsystem, still gotta wait for yuval and alon to finish cadding and shit.
- [ ] Tune the shooter and turret PID loops, and the SOTM calculations.
- [ ] Try testing a fusion localizer when we get the field.
