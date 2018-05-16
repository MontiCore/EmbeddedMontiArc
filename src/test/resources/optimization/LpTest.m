/*
 * ******************************************************************************
 * MontiCore Language Workbench, www.monticore.de
 * Copyright (c) 2018, MontiCore, All rights reserved.
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

package optimization;

// transportation problem example (linear)
script LpTest
    // define problem
    Q m = 3;
    Q n = 2;

    // define A, b
    Q^{m, 1} A = [45; 60; 35];
    Q^{n, 1} b = [50; 60];

    // cost matrix
    Q ^{m, n} c = [3 2; 1 5; 5 4];

    // minimization problem
    minimize(Q^{m, n} x)
        Q y = c(1,1) * x(1,1) + c(2,1) * x(2,1) + c(3,1) * x(3,1) + c(1,2) * x(1,2) + c(2,2) * x(2,2) + c(3,2) * x(3,2);
    subject to
        // sum(X, 2) == A;
        x(1,1) + x(1,2) == A(1);
        x(2,1) + x(2,2) == A(2);
        x(3,1) + x(3,2) == A(3);
        // sum(X, 1) == b;
        x(1,1) + x(2,1) + x(3,1) == b(1);
        x(1,2) + x(2,2) + x(3,2) == b(2);
        //
        x >= 0;
    end
end
