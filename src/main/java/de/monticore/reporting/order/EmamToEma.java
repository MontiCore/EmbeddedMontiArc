package de.monticore.reporting.order;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcbehavior._ast.ASTBehaviorImplementation;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;
import de.monticore.reporting.testCocos.helper.CheckCoCoResult;

import java.util.List;

public class EmamToEma {

    public static void convertToEma(List<CheckCoCoResult> models){
        int z = 0;
        for (CheckCoCoResult model: models) {
            System.out.println(++z);
            convertToEma(model);
        }
    }

    public static void convertToEma(CheckCoCoResult model){
        if (model.getFileType().toUpperCase().equals("EMA")) return;
        else if(!model.getFileType().toUpperCase().equals("EMAM")) return; // ERROR
        else if(model.getParsed() != 1) return; //ERROR

        ASTEMAMCompilationUnit ast = model.getCompilationUnit();

        boolean atomic = false;
        for(ASTElement element: ast.getEMACompilationUnit().getComponent().getBody().getElements()){
            if(element instanceof ASTBehaviorImplementation) atomic = true;
        }
        model.setAtomic(atomic);

        if(atomic){
            model.setEmaToPrint(EmamToEmaPrettyPrinter.prettyPrint(ast));
        }
    }

    public static void convertToEmaSimple(List<CheckCoCoResult> models) {
        int z = 0;
        for (CheckCoCoResult model: models) {

            model.setEmaToPrint(SimpleEmamToEmaPrinter.getEmaString(model.getPathToFile()));
        }
    }

}
