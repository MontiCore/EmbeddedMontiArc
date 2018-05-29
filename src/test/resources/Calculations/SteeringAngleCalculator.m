/*
 * ******************************************************************************
 * MontiCore Language Workbench, www.monticore.de
 * Copyright (c) 2017, MontiCore, All rights reserved.
 *
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * ******************************************************************************
 */

package calculations;

script SteeringAngleCalculator
     Q(-90:90)^{2} x;
     Q(-180:180)^{2} y;
     Q(-90:90) gpsX;
     Q(-180:180) gpsY;
     Q(-180:180) orientation;
     Q(-180:180) currentSteeringAngle;
     Q(-180:180) minSteeringAngle;
     Q(-180:180) maxSteeringAngle;
     Q(-180:180) newSteeringAngle;
     
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

     if globalOrientation > M_PI
         globalOrientation -= 2 * M_PI;
     end;

     Q orientedDistance = distance;
     Q angleTowardsTrajectory = atan(orientedDistance / 2);
     Q orientationOfTrajectory;


     Q v1 = x(2) - x(1);
     Q v2 = y(2) - y(1);
     Q cosineAngle = v2 / sqrt(v1 * v1 + v2 * v2);
     Q angle = acos(cosineAngle);

     if (v1 > 0)
         orientationOfTrajectory = -1 * angle;
     else
         orientationOfTrajectory = angle;
     end;
     Q angleTrajectoryAndCarDirection = orientationOfTrajectory - globalOrientation;

     //the resulting angle is the angle needed to steer the car parallel to the trajectory
     // plus the angle towards the trajectory
     Q finalAngle = angleTrajectoryAndCarDirection + angleTowardsTrajectory;

     //correct angle
     if (finalAngle > M_PI)
         finalAngle -= 2 * M_PI;
     elseif (finalAngle < -1 * M_PI)
          finalAngle += 2 * M_PI;
     end;

     newSteeringAngle = finalAngle*(180/M_PI);

     //correct angle depending on car
        angle = newSteeringAngle;
        if angle < minSteeringAngle
            angle = minSteeringAngle;
        elseif angle > maxSteeringAngle
            angle = maxSteeringAngle;
        end;
        newSteeringAngle = -1 *angle;
end
