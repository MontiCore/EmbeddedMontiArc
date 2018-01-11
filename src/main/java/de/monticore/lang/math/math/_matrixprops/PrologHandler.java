/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.math.math._matrixprops;


import alice.tuprolog.*;
import de.se_rwth.commons.logging.Log;

import java.io.FileInputStream;
import java.util.ArrayList;

/**
 * Created by Philipp Goerick on 07.09.2017.
 *
 * Handles Prolog Queries and Database
 */

public class PrologHandler {
    private final String fileName = "Matrixprops.pl";
    private final String filePath = "src/main/resources/";
    private Theory pcd;
    private Prolog engine;

    public PrologHandler(){
        try {
            pcd = new Theory(new FileInputStream(filePath + fileName));
        }catch (Exception ex){
            Log.error(ex.getMessage());
        }

        engine = new Prolog();

    }

    public void addClause(String str){
        str = str + ".\n";
        try {
            pcd.append(new Theory(str));
        }catch (Exception ex){
            Log.error(ex.getMessage());
        }
    }

    public void removeClauses(){
        engine.clearTheory();
        try {
            pcd = new Theory(new FileInputStream(filePath + fileName));
        }catch (Exception ex){
            Log.error(ex.getMessage());
        }
    }

    public ArrayList<String> getSolution(String str){
        ArrayList<String> res = new ArrayList<>();
        try{
            engine.addTheory(pcd);
            SolveInfo info = engine.solve(str);
            res.add(info.toString());
        }catch (Exception ex){
            Log.error(ex.getMessage());
        }
        return res;
    }
}
