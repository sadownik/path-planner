# Model Documentation - Reflection

This document is provided to meet the last rubric criteria. It's supposed to explain how the path finding works. The trajectory generation principle is adopted from the lecture video. It is basically extended by the function `stateMachine()` that decides which lane to take and how to adjust the speed.

## Trajectory generation

The trajectory generator from the class puts the current cars localization data and the previous step in a vector. based on the map_waypoints ahead it generates three new waypoints ahead and also packs them into the vector. These points are used to fit a spline. After that a target point on the spline is defined. 50 points in between are shifted in the right coordinate system and then passed to the simualtor as points to drive to. 

## The state machine function

Up until line 82 in `sensor_fusion()` the distance to the car ahead is calculated. This is done by filtering the vehicles in the vector, so that there are only the ones left that are in the same lane and ahead. To prevent it "locking on" to a car that is further ahead there is a check for the car that is the nearest.

In a similar way there are checks for cars in the lane to the right or to the left (line 87 to 118). We then set flags `left_possible` and `right_possible` if it is possible to change lanes based on the current lane (can't go left if you are on lane 0 for example).  If a car is in front of us, we raise the flag `car_in_front`.

Now, we define the states for our state machine and put in the logic when to change states.
```c++
enum state {HOLD_LANE, KEEP_DISTANCE, CHANGE_LEFT, CHANGE_RIGHT};

state CurrentState;
  if(car_in_front){
    CurrentState = KEEP_DISTANCE;
  }
  else{
    CurrentState = HOLD_LANE;
  }


timestamp+=1;
if(left_possible && timestamp>100 && car_in_front){
  CurrentState = CHANGE_LEFT;
}
else if(right_possible && timestamp>100 && car_in_front){
  CurrentState = CHANGE_RIGHT;
}
```

Basically, if we have a car in front of us we go into the KEEP_DISTANCE state where we manage the distance. If the other lanes are free, we will engage a lane change. To slow down erratic lane changing we allow a lane change every 100 "timestamps" (loops). The behaviour for the state is defined using a switch construct.

```c++
switch (CurrentState) {

  case HOLD_LANE:
    //accelerate if too slow
    if(ref_vel < 49.5){
      ref_vel += 0.224;
    }
    break;

  case KEEP_DISTANCE:

    k = 0.2;
    // calculate vref based on distance
    ref_vel = 49.5 * k*acc_dist/(1 + abs(k* acc_dist));
    // dont let speed be negative
    if(ref_vel<=0){
      ref_vel = 0;
    }
    //sanity check
    if(ref_vel>49.5){
      ref_vel = 49.5;
    }

    acc_current = ref_vel_alt-ref_vel;
    //override ref_vel and brake with maximum allowed negative accerlation if
    //there is a sudden decrease in distance
    if(acc_current>0.224){
      ref_vel =ref_vel_alt - 0.224;
    }
    // also limit positive acceleration
    else if(acc_current<0.224){
      ref_vel =ref_vel_alt + 0.224;
    }
    break;

  case CHANGE_LEFT:

    lane -=1;
    timestamp = 0;
  break;

  case CHANGE_RIGHT:

    lane +=1;
    timestamp = 0;

  break;
}
```
The HOLD_LANE state is defined by a acceleration to the maximum allowed speed. The KEEP_DISTANCE state calculates a velocity reference as a function of the distance to the car in front. To counter sudden changes in acceleration if a car merges before or the car in front leaves the lane, we limit the accerlation if the function outputs an acceleration that is too high. Both states left either increase or decrease the lane index and reset the timestamp. Lane index and reference speed are then passed back to the trajectory generating algorithm.
