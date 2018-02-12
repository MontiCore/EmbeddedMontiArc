package de.monticore.lang.monticar.generator.rosmsg;

import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class GeneratorRosMsg {

    public List<File> generate(MCTypeReference<? extends MCTypeSymbol> typeReference){
        if(!typeReference.existsReferencedSymbol()) return new ArrayList<>();
        ArrayList<File> files = new ArrayList<>();


        MCTypeSymbol type = typeReference.getReferencedSymbol();
        if(type.isKindOf(MCASTTypeSymbol.KIND)){
            //Not needed, std_msgs already generated
        }


        return files;
    }

    public String getRosType(MCTypeReference<? extends MCTypeSymbol> typeReference){
        MCTypeSymbol type = typeReference.getReferencedSymbol();
        if(type.isKindOf(MCASTTypeSymbol.KIND)){
            MCASTTypeSymbol mcastTypeSymbol = (MCASTTypeSymbol) type;
            if(mcastTypeSymbol.getName().equals("Q")){
                return "std_msgs/Float64";
            }else if(mcastTypeSymbol.getName().equals("Z")){
                return "std_msgs/Int32";
            }else if(mcastTypeSymbol.getName().equals("B")){
                return "std_msgs/Bool";
            }else{
                Log.error("Case not handled! MCASTTypeSymbol " + mcastTypeSymbol.getName());
            }
        }



        Log.error("Case not handled! MCTypeReference " + typeReference);
        return null;
    }

}
