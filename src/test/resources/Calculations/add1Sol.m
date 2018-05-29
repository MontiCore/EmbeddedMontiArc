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

script add1Sol
    Q^{2,2} A = [1 2 ;5 4] ;
    Q^{3,3} Bmat = [30 36 42 ;66 81 96 ;102 126 150 ] ;
    Q^{1,2} A = [4 6 ];
    Q^{2,2} B = [17 9 ;33 6 ];
    Q^{1,2} A = [8 N 13 N ];
    A = [1 8 ];
    Q^{2,1} C = [5 ;4 ];
    B = [5 6 ;7 8 ];
    C = [17 ;39 ];
    Q^{1,3} D = [1 1 1 ];
    Q^{2,3} E = [-31 N, 28 N, 22 N ;-50 N, -2 N, 2 N ];
    Q^{3,3} F = [1 4 7 ;2 5 8 ;3 6 9 ];
    Q g = 3 m;
    Q h = 9 m^4/ 250000;
    Q i = 15;
    Q^3 K = [3/11 ;1/11 ;7/22 ];
end
