// (c) https://github.com/MontiCore/monticore 

package calculations;

script DegreeTest
    Q(-90°:90°)^2 x;
    Q(-180°:180°)^2 y;
    Q(-90°:90°) gpsX;
    Q(-180°:180°) gpsY;
    Q(-180°:180°) orientation;
    Q(-180°:180°) currentSteeringAngle;
    Q(-180°:180°) minSteeringAngle;
    Q(-180°:180°) maxSteeringAngle;
    Q(-180°:180°) newSteeringAngle;
	
     Q distance;
     Q res =(y(2) - y(1))*gpsX;
     res -= (x(2) - x(1))*gpsY;
     res += x(2) * y(1);
     res -= y(2) * x(1);
     Q xDiff = x(1) - x(2);
     Q yDiff = y(1) - y(2);
     res /= sqrt(xDiff*xDiff+yDiff*yDiff);

     distance = res;

     //calculate base steering angle
     Q globalOrientation = orientation*(M_PI/180);
     if (globalOrientation > M_PI)
         globalOrientation -= 2 * M_PI;
     end

     Q orientedDistance = distance;
     Q angleTowardsTrajectory = atan(orientedDistance / 2);
     Q orientationOfTrajectory;


     Q v1 = x(2) - x(1);
     Q v2 = y(2) - y(1);
     Q cosineAngle = v2 / sqrt(v1 * v1 + v2 * v2);
     Q angle = acos(cosineAngle);

     if (v1 > 0)
         orientationOfTrajectory = -angle;
     else
         orientationOfTrajectory = angle;
     end
     Q angleTrajectoryAndCarDirection = orientationOfTrajectory - globalOrientation;

     //the resulting angle is the angle needed to steer the car parallel to the trajectory
     // plus the angle towards the trajectory
     Q finalAngle = angleTrajectoryAndCarDirection + angleTowardsTrajectory;

     //correct angle
     if (finalAngle > M_PI)
         finalAngle -= 2 * M_PI;
     elseif (finalAngle < -M_PI)
          finalAngle += 2 * M_PI;
     end

     newSteeringAngle = finalAngle*(180/M_PI);

     //correct angle depending on car
        angle = newSteeringAngle;
        if angle < minSteeringAngle
            angle = minSteeringAngle;
        elseif angle > maxSteeringAngle
            angle = maxSteeringAngle;
        end
        newSteeringAngle = -angle;
end