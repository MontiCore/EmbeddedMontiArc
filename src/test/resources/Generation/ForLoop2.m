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

script ForLoop2
  Q(0 m : 1000 m)^{5} c = [1 m, 3 m , 5 m, 7 m, 9 m]
  Q x = 0
  Q(0 m^2 : 1000 m^2) y = 0 m*m
  Q(0 m : 1000 m) z = 0 m

  for i = c
   for j = c
     y+=j*i ;
     z += c(x+0);
    end
   x+=1;
  end
end
