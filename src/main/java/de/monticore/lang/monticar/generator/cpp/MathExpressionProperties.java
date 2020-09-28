/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

/**
 * @author Ahmed Diab
 */
public class MathExpressionProperties {
    public Level pre = Level.ARMA;
    public Level suc = Level.ARMA;

    enum Level {
        CV,
        ARMA
    }

    public void setPreToCV(){
        this.pre = Level.CV;
    }

    public void setSucToCV(){
        this.suc = Level.CV;
    }

    public boolean isPreCV(){
        return this.pre == Level.CV;
    }

    public boolean isSucCV(){
        return this.suc == Level.CV;
    }
}
