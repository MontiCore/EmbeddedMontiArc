package visualization.main;

import visualization.basics.*;
import visualization.doors.*;
import visualization.lights.*;
import visualization.movement.*;
import visualization.feature.*;

component SDCS {
    port
        in Double time,

        // Distances of the sensors to the track
        in (0m:2m) fl, // frontleft
        in (0m:2m) fr, // frontright
        in (0m:2m) slf, // sideleftfront
        in (0m:2m) slb, // sideleftback
        in (0m:2m) srf, //siderightfront
        in (0m:2m) srb, //siderightback

        in (0km/h : 250km/h) velocity,
        in N x,
        in N y,

        out (-180°:180°) steering,
        out (-2m/s^2:2m/s^2) acceleration,
        out Boolean lightStatus,
        out Boolean indicatorStatus,
        out Boolean triggerStatus,
        out Boolean brakelightStatus,
        out Boolean doorStatus;


    instance SteeringControl steeringControl;
    instance ConstantVelocity constantVelocity;
    instance IndicatorStatus indStatus;
    instance LightTimer lightTimer;
    instance DoorStatus ds;
    instance GameOverTrigger trigger;

    connect time->constantVelocity.time;
    connect velocity->constantVelocity.velocity;
    connect fl->steeringControl.fl;
    connect fr->steeringControl.fr;
    connect slf->steeringControl.slf;
    connect slb->steeringControl.slb;
    connect srf->steeringControl.srf;
    connect srb->steeringControl.srb;
    connect time->indStatus.time;
    connect velocity->ds.velocity;
    connect time->lightTimer.time;
    connect x->trigger.x;
    connect y->trigger.y;

    connect constantVelocity.acceleration->acceleration;
    connect steeringControl.steering->steering;
    connect indStatus.status->indicatorStatus;
    connect ds.status->doorStatus;
    connect lightTimer.status->lightStatus;
    connect trigger.status->triggerStatus;
}