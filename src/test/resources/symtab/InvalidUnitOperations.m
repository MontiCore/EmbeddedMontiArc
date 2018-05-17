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

package symtab;

script InvalidUnitOperations

Q(0m : 10m)^{1,2} a = [1 m, 2 m];
Q(0m : 10m)^{2,2} b = [1 m, 2 m;1 m , 2 m];


Q(0m : 100m)^{1,2} c = a*b; // invalid unit --> a*b returns m^2 as unit

Q(-20m : 100m)^{2,2} d =[1 m, 2 m;2m ,1 m];
Q(0 : 10kg)^{1,2} e =[1 kg, 2 kg];

Q^{1,2} f =e*d; // unit error

Q(0 km/h : 100m/s)^{3} matrix = [1 m/s, 2 km/h ,3 m/s^2]; // unit error

Q(0 : 7000 m)^{3,4} matrix1 = [1 mm 2 mm 3 mm 4 cm ;5 dm 6 m 7 km 8 in;9 ft 10 m 11 nm 12 cm];
Q(-20 m/s : 100 km/h)^{3,4} matrix2 = [1 m/s 2 km/h 3 m/s 4 m/s; 5 m/s 6 m/s 7 km/h 8 m/s; 9 m/s 10 m/s 11 m/s 12 km/h];
matrix1 + matrix2 // unit error

end





