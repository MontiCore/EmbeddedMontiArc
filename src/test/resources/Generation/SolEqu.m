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

package Generation;
script SolEqu

  Q^{3,1} A = [3 6 2; 1 2 8; 7 9 4] \\ [2;3;4];
  Q^{3,1} Bmat = ([3/2 6/2 2/2; 1/2 2/2 8/2; 7/2 9/2 4/2]+[3/2 6/2 2/2; 1/2 2/2 8/2; 7/2 9/2 4/2]) \\ [2;3;4];
  //Q(0 m : 100 m)^{3,1} C = [3 m 6 m 2 m; 1 m 2 m 8 m; 7 m 9 m 4 m] \\ [2 m*m;3 m*m;4 m*m];

end
