package de.monticore.lang.monticar.generator.rosmsg;

import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;
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
        List<File> res = new ArrayList<>();
        List<FileContent> fileContents = generateStrings(typeReference);
        for (FileContent fileContent : fileContents) {
            File file = new File(path + "/" + fileContent.getFileName());
            FileUtils.write(file, fileContent.getFileContent());
            res.add(file);
        }
        return res;
    }

    public List<FileContent> generateStrings(MCTypeReference<? extends MCTypeSymbol> typeReference) {
        //is typeReference basic type or struct?
        if (getRosType(currentPackageName, typeReference) != null) {
            MCTypeSymbol type = typeReference.getReferencedSymbol();
            List<FileContent> res = new ArrayList<>();
            if (type instanceof StructSymbol) {
                res.addAll(generateStruct(currentPackageName, (StructSymbol) type));
            }
            return res;
        }

        Log.error("Cannot generate .msg file(s) for " + typeReference);
        return new ArrayList<>();
    }

    public List<FileContent> generateStruct(String packageName, StructSymbol structSymbol) {
        if (alreadyGenerated.contains(packageName + "/" + getTargetName(structSymbol))) {
            Log.debug("Struct " + structSymbol + " was already generated!", "structGeneration");
            return new ArrayList<>();
        }
        alreadyGenerated.add(packageName + "/" + getTargetName(structSymbol));

        List<FileContent> fileContents = new ArrayList<>();

        String definition = structSymbol.getStructFieldDefinitions().stream()
                .filter(sfds -> sfds.getType().existsReferencedSymbol())
                .map(sfds -> getInMsgRosType(currentPackageName, sfds.getType().getReferencedSymbol()) + " " + sfds.getName())
                .collect(Collectors.joining("\n"));

        FileContent fc = new FileContent();
        fc.setFileName(getTargetName(structSymbol) + ".msg");
        fc.setFileContent(definition);
        fileContents.add(fc);

        //recursively generated needed .msg from nested structs
        for (StructFieldDefinitionSymbol sfds : structSymbol.getStructFieldDefinitions()) {
            if (sfds.getType().existsReferencedSymbol()) {
                if (sfds.getType().getReferencedSymbol() instanceof StructSymbol) {
                    StructSymbol referencedSymbol = (StructSymbol) sfds.getType().getReferencedSymbol();
                    fileContents.addAll(generateStruct(packageName, referencedSymbol));
                }
            }
        }

        return fileContents;
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
        String typeName = type.getName();

        if (typeName.equals("Q")) {
            RosMsg tmpMsg = new RosMsg("std_msgs/Float64");
            tmpMsg.addField(new RosField("data", new RosType("float64")));
            return tmpMsg;
        } else if (typeName.equals("Z")) {
            RosMsg tmpMsg = new RosMsg("std_msgs/Int32");
            tmpMsg.addField(new RosField("data", new RosType("int32")));
            return tmpMsg;
        } else if (typeName.equals("B")) {
            RosMsg tmpMsg = new RosMsg("std_msgs/Bool");
            tmpMsg.addField(new RosField("data", new RosType("bool")));
            return tmpMsg;
        } else if (typeName.equals("CommonMatrixType") && type.isKindOf(MCASTTypeSymbol.KIND)) {
            ASTCommonMatrixType matrixType = (ASTCommonMatrixType) ((MCASTTypeSymbol) type).getAstType();
            String tmpMsgName = "";
            String tmpTypeName = "";
            if (matrixType.getElementType().isRational()) {
                tmpMsgName = "std_msgs/Float64MultiArray";
                tmpTypeName = "float64";
            } else if (matrixType.getElementType().isWholeNumber()) {
                tmpMsgName = "std_msgs/Int32MultiArray";
                tmpTypeName = "int32";
            } else if (matrixType.getElementType().isBoolean()) {
                tmpMsgName = "std_msgs/ByteMultiArray";
                tmpTypeName = "byte";
            } else {
                Log.error("Matrix type not supported: " + matrixType);
            }
            //TODO: refactor
            return getMultMatrixRosMsg(tmpMsgName, tmpTypeName);
        }

        if (type instanceof StructSymbol) {
            StructSymbol structSymbol = (StructSymbol) type;
            return createMsgForStruct(packageName, structSymbol);
        }
        Log.error("Case not handled! MCTypeReference " + typeReference);
        return null;
    }

    private static RosMsg getMultMatrixRosMsg(String msgName, String tmpTypeName) {
        RosMsg tmpMsg = new RosMsg(msgName);

        //uint32 data_offset
        RosMsg multiArrayLayout = new RosMsg("MultiArrayLayout");
        multiArrayLayout.addField(new RosField("data_offset", new RosType("uint32")));

        //MultiArrayDimension[] dim
        RosMsg mutliArrayDimMsg = new RosMsg("MultiArrayDimension");

        //string label
        mutliArrayDimMsg.addField(new RosField("label", new RosType("string")));
        //uint32 size
        mutliArrayDimMsg.addField(new RosField("size", new RosType("uint32")));
        //uint32 stride
        mutliArrayDimMsg.addField(new RosField("stride", new RosType("uint32")));

        RosField multiArrayDimField = new RosField("dim", mutliArrayDimMsg);
        multiArrayDimField.setArray(true);

        multiArrayLayout.addField(multiArrayDimField);

        RosField fieldData = new RosField("data", new RosType(tmpTypeName));
        fieldData.setArray(true);
        RosField layoutField = new RosField("layout", multiArrayLayout);

        tmpMsg.addField(fieldData);
        tmpMsg.addField(layoutField);

        return tmpMsg;
    }

    private static String getTargetName(StructSymbol structSymbol) {
        return structSymbol.getFullName().replace(".", "_");
    }

    private static String getFullTargetName(String packageName, StructSymbol structSymbol) {
        return packageName + "/" + getTargetName(structSymbol);
    }
}
