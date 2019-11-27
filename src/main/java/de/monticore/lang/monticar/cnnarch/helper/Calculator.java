/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.helper;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;

public class Calculator {

    private static Calculator instance = null;

    private ScriptEngine engine;

    private Calculator() {
        ScriptEngineManager manager = new ScriptEngineManager();
        engine = manager.getEngineByExtension("js");
    }

    public static Calculator getInstance(){
        if (instance == null){
            instance = new Calculator();
        }
        return instance;
    }

    public static void clear(){
        instance = null;
    }


    public Object calculate(String expression){
        try {
            return engine.eval(expression);
        }
        catch (ScriptException e){
            throw new IllegalArgumentException("Calculation error in the expression: " + expression);
        }
    }

}
