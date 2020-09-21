/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event;

import de.monticore.lang.embeddedmontiarcdynamic.cocos.EventCoCos;
import de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTEventCompilationUnit;
import de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTEventNode;
import de.monticore.lang.embeddedmontiarcdynamic.event._cocos.EventCoCoChecker;
import de.monticore.lang.embeddedmontiarcdynamic.event._parser.EventParser;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.ComponentEventSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.nio.file.FileSystemNotFoundException;
import java.nio.file.Paths;
import java.util.Optional;

import static junit.framework.TestCase.fail;

public class CocoTest extends AbstractTest{

    protected static final String basePath = "./src/test/resources/test/event/cocos";



    @Test
    public void Test_01_EventCapitalized(){

        innerTest("eventCapitalized");
    }

    protected boolean  innerTest(String name){
        Log.enableFailQuick(false);
        Optional<ComponentEventSymbol> comp = Optional.empty();
        try {
            EventParser ep = new EventParser();
            Optional<ASTEventCompilationUnit> valid = ep.parse(Paths.get(basePath, "valid/", name+".event").toString());
            if(!valid.isPresent()){
                Log.error("01: There were unexpected parser errors");
                fail("01: There were unexpected parser errors");
                return false;
            }
            Optional<ASTEventCompilationUnit> invalid = ep.parse(Paths.get(basePath, "invalid/", name+".event").toString());
            if(!invalid.isPresent()){
                Log.error("02: There were unexpected parser errors");
                fail("02: There were unexpected parser errors");
                return false;
            }

            EventCoCos.createChecker().checkAll(valid.get());

            EventCoCos.createChecker().checkAll(invalid.get());

            if(  Log.getErrorCount() != 1){
                fail(String.format("Wrong number of errors: %d, expected: %d", Log.getErrorCount(), 1));
                Log.getFindings().clear();
            }
        }catch (Exception ex){
            ex.printStackTrace();
            return false;
        }

        return true;
    }
}
