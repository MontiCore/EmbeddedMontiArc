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

script ExistingOptimizationVariable
    // 1. scalar
    Q x = 3;
    minimize(x)
        Q y = 2 * x + 1;
    subject to
        -1 <= x <= 1;
    end
    // 2. matrix
    Q^{3,3} a = zeros(3,3);
    minimize(a)
        Q b = a * a';
    subject to
        -10 <= x <= 10;
    end
    // 3. substituted
    Q squared = a * a;
    minimize(a)
        Q b = squared * squared';
    subject to
        -10 <= x <= 10;
    end
end
