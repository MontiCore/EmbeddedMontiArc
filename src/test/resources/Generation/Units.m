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
script Units
  Q(0 m : 10 m) A = 5 m;
  Q(0 m : 10 m) Bmat = 1 m;
  Q(0 m : 10 m)^{2} Cmat = [1 m, 2 m];
  Q(0 m*s : 10 m*s) F = 2 m * 1 s;
  Q(0 m^2 : 10 m^2) D = 1 m*m;
  Q(0 m^4 : 1000 m^4) E = ((1 m + 2 m)*(3 m + 4 m))^2;

end
