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

script Units2
  Q(0 m : 10 m)^{1,2} A = [1 m, 2 m];
  Q(0 m : 10 m)^{2,1} matB = [1 m; 2 m];
  Q(0 m^2 : 100 m^2)^{1,1} matC = A*matB;
  Q(0 m : 10 m) D = 2 m;
  Q(0 m : 10 m) E = 5 m;
  Q(0 m^2 : 200 m^2) F = D*E + 3 m^2;

end
