/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._ast;

import java.util.Optional;

public class ASTCTV extends ASTCTVTOP {

    public ASTCTV(){
        super();
    }

    public  ASTCTV (/* generated by template ast.ConstructorParametersDeclaration*/
            Optional<ASTCompareToValueGreater> compareToValueGreater
            ,
            Optional<de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTCompareToValueGreaterEquals> compareToValueGreaterEquals
            ,
            Optional<de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTCompareToValueLower> compareToValueLower
            ,
            Optional<de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTCompareToValueLowerEquals> compareToValueLowerEquals
            ,
            Optional<de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTCompareToValueNotEquals> compareToValueNotEquals

    ) {
        super(compareToValueGreater, compareToValueGreaterEquals, compareToValueLower, compareToValueLowerEquals, compareToValueNotEquals);
    }

    public ASTCompareToValue getCompareToValue(){
        if(this.isPresentCompareToValueGreater()){
            return (this.getCompareToValueGreater());
        }else if(this.isPresentCompareToValueGreaterEquals()){
            return this.getCompareToValueGreaterEquals();
        }else if(this.isPresentCompareToValueLower()){
            return this.getCompareToValueLower();
        }else if(this.isPresentCompareToValueLowerEquals()){
            return this.getCompareToValueLowerEquals();
        }else if(this.isPresentCompareToValueNotEquals()){
            return this.getCompareToValueNotEquals();
        }
        return null;
    }

}
