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

package matrix;

script M2dUnit
    Q(0m : 10km)^{3,4} matrix1 = [1 mm 2 mm 3 mm 4 cm ;5 dm 6 m 7 km 8 inch;9 ft 10 mi 11 nm 12 um];
    Q(1:3 m/s)^{3} matrix1 = [1 m/s 2 km/h 3 m/s];
    Q(0 : 10 m/s^2) a = 0.5 m/s^2;
    Q(1 : 10 N)^{2} f = [1 N; 2 kg*m/s^2];
    Q(0 : 2 : 6)^{3} v = [2 + 4 4 sin(0)]; // is array [6 4 0]
end
