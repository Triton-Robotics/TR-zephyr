#include "HeroShooterSubsystem.h"
#include "util/motor/DJIMotor.h"

HeroShooterSubsystem::HeroShooterSubsystem(config cfg):
    flywheelL({
        cfg.top_can_device,
        cfg.flywheelL_id,
        cfg.canBusTopFeed,
        M3508_FLYWHEEL,
        "Left Flywheel",
        cfg.flywheelL_PID
    }),
    flywheelR({
        cfg.top_can_device,
        cfg.flywheelR_id,
        cfg.canBusTopFeed,
        M3508_FLYWHEEL,
        "Right Flywheel",
        cfg.flywheelR_PID
    }),
    indexer({
        cfg.indexer_can_device,
        cfg.indexer_id,
        cfg.canBusIndexer,
        M3508,
        "Indexer",
        cfg.indexer_PID_vel,
        cfg.indexer_PID_pos
    }),
    feeder({
        cfg.top_can_device,
        cfg.feeder_id,
        cfg.canBusTopFeed,
        M2006,
        "Feeder",
        cfg.feeder_PID
    })
{
    // initialize all other vars
    shoot = OFF;
    shootReady = true;

    barrel_heat_limit = cfg.heat_limit;
    invert_flywheel = cfg.invert;

    shooter_time = now_us();
}


void HeroShooterSubsystem::setState(ShootState shoot_state)
{
    shoot = shoot_state;
}


void HeroShooterSubsystem::periodic(int curr_heat, int heat_limit) 
{
    barrel_heat = curr_heat;
    barrel_heat_limit = heat_limit;

    if (shoot == OFF) 
    {
        flywheelL.setSpeed(0);
        flywheelR.setSpeed(0);
        indexer.setPower(0);
        shootReady = true;
        shooter_time = 0;
        feeder.setSpeed(0);
    }
    else if (shoot == FLYWHEEL)
    {
        if (!invert_flywheel) {
            flywheelL.setSpeed(-HERO_FLYWHEEL_VELO);
            flywheelR.setSpeed(HERO_FLYWHEEL_VELO);
        }
        else {
            flywheelL.setSpeed(HERO_FLYWHEEL_VELO);
            flywheelR.setSpeed(-HERO_FLYWHEEL_VELO);
        }

        indexer.pidSpeed.feedForward = 0;
        indexer.setSpeed(0);
        shootReady = true;
        if(barrel_heat_limit < 10 || barrel_heat < barrel_heat_limit - 110) {
            shooter_time = now_us();
        }
        feeder.setSpeed(0);
    }
    else if (shoot == SHOOT)
    {
        
        if (!invert_flywheel) {
            flywheelL.setSpeed(-HERO_FLYWHEEL_VELO);
            flywheelR.setSpeed(HERO_FLYWHEEL_VELO);
        }
        else {
            flywheelL.setSpeed(HERO_FLYWHEEL_VELO);
            flywheelR.setSpeed(-HERO_FLYWHEEL_VELO);
        }

        if (abs(flywheelR >> VELOCITY) < fabs(HERO_FLYWHEEL_VELO * 0.75) || abs(flywheelL >> VELOCITY) < fabs(HERO_FLYWHEEL_VELO * 0.75)) {
            return;
        }

        if ((now_us() - shooter_time)/1000 < 200){
            feeder.setSpeed(7000);
        } else {
            feeder.setSpeed(0);
        }
        if ((now_us() - shooter_time)/1000 < 150){
            indexer.setSpeed(6000);
        }else if ((now_us() - shooter_time)/1000 > 500 && (now_us() - shooter_time)/1000 < 650){
            indexer.setPower(-8000);
        } else {
            indexer.setPower(0);
        }
    }
}