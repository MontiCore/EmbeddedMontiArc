package de.monticore.lang.monticar.generator.rosmsg;

import de.monticore.lang.monticar.struct._symboltable.StructFieldDefinitionSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class GeneratorRosMsg {
    private String path;
    private String currentPackageName;
    private Set<String> alreadyGenerated = new HashSet<>();


    public void setTarget(String path, String packageName) {
        if (path.endsWith("/")) {
            this.path = path + "/";
        } else {
            this.path = path;
        }

        this.currentPackageName = packageName;
    }

    public List<File> generate(MCTypeReference<? extends MCTypeSymbol> typeReference) throws IOException {
        //is typeReference basic type or struct?
        if (getRosType(currentPackageName, typeReference) != null) {
            MCTypeSymbol type = typeReference.getReferencedSymbol();
            List<File> res = new ArrayList<>();
            if (type instanceof StructSymbol) {
                res.addAll(generateStruct(currentPackageName, (StructSymbol) type));
            }
            return res;
        }

        Log.error("Cannot generate .msg file(s) for " + typeReference);
        return new ArrayList<>();
    }

    public List<File> generateStruct(String packageName, StructSymbol structSymbol) throws IOException {
        if (alreadyGenerated.contains(packageName + "/" + getTargetName(structSymbol))) {
            Log.debug("Struct " + structSymbol + " was already generated!", "structGeneration");
            return new ArrayList<>();
        }
        alreadyGenerated.add(packageName + "/" + getTargetName(structSymbol));

        List<File> files = new ArrayList<>();

        String definition = structSymbol.getStructFieldDefinitions().stream()
                .filter(sfds -> sfds.getType().existsReferencedSymbol())
                .map(sfds -> getInMsgRosType(currentPackageName, sfds.getType().getReferencedSymbol()) + " " + sfds.getName())
                .collect(Collectors.joining("\n"));

        File f = new File(path + "/" + getTargetName(structSymbol) + ".msg");
        FileUtils.write(f, definition);

        //recursively generated needed .msg from nested structs
        for (StructFieldDefinitionSymbol sfds : structSymbol.getStructFieldDefinitions()) {
            if (sfds.getType().existsReferencedSymbol()) {
                if (sfds.getType().getReferencedSymbol() instanceof StructSymbol) {
                    StructSymbol referencedSymbol = (StructSymbol) sfds.getType().getReferencedSymbol();
                    generateStruct(packageName, referencedSymbol);
                }
            }
        }

        return files;
    }

    public static RosMsg createMsgForStruct(String packageName, StructSymbol structSymbol) {
        RosMsg res = new RosMsg(getFullTargetName(packageName, structSymbol));
        structSymbol.getStructFieldDefinitions().stream()
                .filter(sfds -> sfds.getType().existsReferencedSymbol())
                .forEach(sfds -> {
                    MCTypeSymbol referencedSymbol = sfds.getType().getReferencedSymbol();
                    if (referencedSymbol instanceof StructSymbol) {
                        res.addField(new RosField(sfds.getName(), createMsgForStruct(packageName, (StructSymbol) referencedSymbol)));
                    } else {
                        res.addField(new RosField(sfds.getName(), new RosType(getInMsgRosType(packageName, referencedSymbol))));
                    }
                });
        return res;
    }

    private static String getInMsgRosType(String packageName, MCTypeSymbol referencedSymbol) {
        if (referencedSymbol.isKindOf(MontiCarTypeSymbol.KIND)) {
            MontiCarTypeSymbol mcastTypeSymbol = (MontiCarTypeSymbol) referencedSymbol;
            if (mcastTypeSymbol.getName().equals("Q")) {
                return "float64";
            } else if (mcastTypeSymbol.getName().equals("Z")) {
                return "int32";
            } else if (mcastTypeSymbol.getName().equals("B")) {
                return "bool";
            } else {
                Log.error("Case not handled! MCASTTypeSymbol " + mcastTypeSymbol.getName());
            }
        }

        if (referencedSymbol instanceof StructSymbol) {
            StructSymbol structSymbol = (StructSymbol) referencedSymbol;
            return packageName + "/" + getTargetName(structSymbol);
        }

        Log.error("Case not handled! MCTypeReference " + referencedSymbol);
        return null;
    }

    public static RosMsg getRosType(String packageName, MCTypeReference<? extends MCTypeSymbol> typeReference) {
        MCTypeSymbol type = typeReference.getReferencedSymbol();
        if(type.isKindOf(MCASTTypeSymbol.KIND)){
            MCASTTypeSymbol mcastTypeSymbol = (MCASTTypeSymbol) type;
            if(mcastTypeSymbol.getName().equals("Q")){
                RosMsg tmpMsg = new RosMsg("std_msgs/Float64");
                tmpMsg.addField(new RosField("data", new RosType("float64")));
                return tmpMsg;
            }else if(mcastTypeSymbol.getName().equals("Z")){
                RosMsg tmpMsg = new RosMsg("std_msgs/Int32");
                tmpMsg.addField(new RosField("data", new RosType("int32")));
                return tmpMsg;
            }else if(mcastTypeSymbol.getName().equals("B")){
                RosMsg tmpMsg = new RosMsg("std_msgs/Bool");
                tmpMsg.addField(new RosField("data", new RosType("bool")));
                return tmpMsg;
            }else{
                Log.error("Case not handled! MCASTTypeSymbol " + mcastTypeSymbol.getName());
            }
        }

        if (type instanceof StructSymbol) {
            StructSymbol structSymbol = (StructSymbol) type;
            return createMsgForStruct(packageName, structSymbol);
        }
        Log.error("Case not handled! MCTypeReference " + typeReference);
        return null;
    }

    private static String getTargetName(StructSymbol structSymbol) {
        return structSymbol.getFullName().replace(".", "_");
    }

    private static String getFullTargetName(String packageName, StructSymbol structSymbol) {
        return packageName + "/" + getTargetName(structSymbol);
    }
}
