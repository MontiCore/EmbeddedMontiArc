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

script MatrixWithMathExpression
  Q B = 5;
  Q C = 3;
  Q^{2,2} A = [1+1,1*2;1-C,1/B];
  A += A^2;
  Q^{3,2} D = ([1,2,3;4,5,6]+[1,2,3;4,5,6])';
end
