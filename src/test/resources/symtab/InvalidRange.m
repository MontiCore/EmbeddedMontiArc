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

script InvalidRange


// Wrong Range at Assignment and (therefore) additionally Range does not fit
Q (11:8) a = 5;

// Range does not fit
// 1
// with new assignment
Q(-3:10) b = 5;
b = -25;

// 2
Q(-5:-3) c = 2;

// 3
// with MathValueReference
Q(-4:3) d = 1 ^ 2;
Q(0:10) e = d ^ 2;

// with multiplied ranges
Q(2:5) f = 4;
Q(6:7) g = 6;
Q(5:10) i1 = f * g; // 4

// with divided ranges // 5
Q(1:7) k = 3;
Q(2:8) l = k / 5;

// with Mod Operator // 6
Q(14:22) m = 15;
Q(4:5) n = m%3;

// with PowerOperator // 7
Q(-6:3) o = 3 ^ 2;

// Matrix Range does not fit

Q(-3:10)^{2,3} matrix1 = [2,1,5;-1,3,5];
Q(-1:1)^{3,1} vector1 = [1;1;1];

// with multiplied ranges // 1
Q(0:10)^{2,1} vector2 = matrix1 * vector1;

// with PowerWise // 2
Q(0:10)^{2,3} matrix2 = matrix1.^2;

// with PowerWise odd // 3
Q(0:10)^{2,3} matrix3 = matrix1.^3;

// with TimesWise // 4
Q(0:10)^{2,3} matrix4 = matrix1.*[1,2,3;4,5,-6];

end
