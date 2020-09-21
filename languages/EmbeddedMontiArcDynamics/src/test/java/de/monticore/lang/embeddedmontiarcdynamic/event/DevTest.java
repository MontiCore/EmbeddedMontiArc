/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event;

import de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTEventCompilationUnit;
import de.monticore.lang.embeddedmontiarcdynamic.event._parser.EventParser;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.ComponentEventSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventExpressionSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Ignore;
import org.junit.Test;

import java.io.IOException;
import java.util.Optional;

@Ignore
public class DevTest extends AbstractTest {

    @Test
    @Ignore
    public void TestDev1() throws IOException {
        Scope symScope = createSymTab("./src/test/resources/test/event");

        EventParser ep = new EventParser();
        Optional<ASTEventCompilationUnit> parsedEvent = ep.parse("./src/test/resources/test/event/devtest/PortAsParameter.event");

        Optional<ComponentEventSymbol> comp = Optional.empty();
        try {
            comp = symScope.<ComponentEventSymbol>resolve(
                    "devtest.PortQIsOne", ComponentEventSymbol.KIND);
        }catch (Exception ex){
            ex.printStackTrace();
        }

        if(comp.isPresent()){
            System.out.println(comp.get().toNiceString());
        }
    }


    @Test
    @Ignore
    public void TestDev2() throws IOException {
        Scope symScope = createSymTab("./src/test/resources/test/event");

        Optional<ComponentEventSymbol> comp = Optional.empty();
        try {
            comp = symScope.<ComponentEventSymbol>resolve(
                    "devtest.eventWithUnit", ComponentEventSymbol.KIND);
        }catch (Exception ex){
            ex.printStackTrace();
        }

        if(!comp.isPresent()){
            Log.error("There were unexpected parser errors");
        }else{
            Log.getFindings().clear();
        }
    }

    @Test
    @Ignore
    public void TestDev3() throws IOException {
        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/event");

        Optional<ComponentEventSymbol> comp = Optional.empty();
        try {
            comp = symScope.<ComponentEventSymbol>resolve(
                    "handling.test1.PortTest", ComponentEventSymbol.KIND);
        }catch (Exception ex){
            ex.printStackTrace();
        }



        if(!comp.isPresent()){
            Log.error("There were unexpected parser errors");
        }else{

            EventExpressionSymbol ees = comp.get().getCondition().expand();

            System.out.println(comp.get().toNiceString());

            System.out.println(ees.getTextualRepresentation());

            Log.getFindings().clear();
        }
    }
}
