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

script add1
    Q^{2,2} A = [1 2; 3 4];
    Q^{3,3} Bmat = [1 2 3; 4 5 6; 7 8 9]^2;
    [1 2] + [3 4]
    [5 6; 9 3] + [12 3; 24 3]
    [3 N 6 N] + [5 N 7 N]
    [1 2].^3
    [5 4]'
    [1 0; 0 1]*[5 6; 7 8]
    [1 2; 3 4]*[5;6]
    [9 23 7] - [8 22 6]
    [12 N 34 N 45 N; 2 N 98 N 73 N] - [43 N 6 N 23 N; 52 N 100 N 71 N]
    [1 2 3;4 5 6;7 8 9 ]'
    1 m + 2 m
    3 m * 4 dm * 1 cm * 3 mm

    A(1,0)+5*2

    Bmat(1:2,1:3)
end
