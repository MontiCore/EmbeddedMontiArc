/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._matrixprops;


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
