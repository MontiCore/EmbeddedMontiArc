/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.rosmsg;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.rosmsg.util.CMakeListsViewModel;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;
import de.monticore.lang.monticar.generator.rosmsg.util.RosMsgTemplates;
import de.monticore.lang.monticar.struct._symboltable.StructFieldDefinitionSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.references.SymbolReference;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class GeneratorRosMsg {
    private String path;
    private String currentPackageName;
    private Set<String> alreadyGenerated = new HashSet<>();
    private boolean ros2mode = false;

    public boolean isRos2mode() {
        return ros2mode;
    }

    public void setRos2mode(boolean ros2mode) {
        this.ros2mode = ros2mode;
    }

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
        if (getRosType(currentPackageName, typeReference, ros2mode) != null) {
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
        if (alreadyGenerated.contains(packageName + "/" + getTargetName(structSymbol, ros2mode))) {
            Log.debug("Struct " + structSymbol + " was already generated!", "structGeneration");
            return new ArrayList<>();
        }
        alreadyGenerated.add(packageName + "/" + getTargetName(structSymbol, ros2mode));

        List<FileContent> fileContents = new ArrayList<>();

        String definition = structSymbol.getStructFieldDefinitions().stream()
                .filter(sfds -> sfds.getType().existsReferencedSymbol())
                .map(sfds -> getInMsgRosType(currentPackageName, sfds.getType().getReferencedSymbol(), ros2mode) + " " + getFieldName(sfds, ros2mode))
                .collect(Collectors.joining("\n"));

        FileContent fc = new FileContent();
        fc.setFileName(getTargetName(structSymbol, ros2mode) + ".msg");
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

    public static RosMsg createMsgForStruct(String packageName, StructSymbol structSymbol, boolean ros2mode) {
        RosMsg res = new RosMsg(getFullTargetName(packageName, structSymbol, ros2mode));
        structSymbol.getStructFieldDefinitions().stream()
                .filter(sfds -> sfds.getType().existsReferencedSymbol())
                .forEach(sfds -> {
                    MCTypeSymbol referencedSymbol = sfds.getType().getReferencedSymbol();
                    if (referencedSymbol instanceof StructSymbol) {
                        res.addField(new RosField(getFieldName(sfds, ros2mode), createMsgForStruct(packageName, (StructSymbol) referencedSymbol, ros2mode)));
                    } else {
                        res.addField(new RosField(getFieldName(sfds, ros2mode), new RosType(getInMsgRosType(packageName, referencedSymbol, ros2mode))));
                    }
                });
        return res;
    }

    private static String getFieldName(StructFieldDefinitionSymbol sfds, boolean ros2mode) {
        if (ros2mode) {
            return sfds.getName().toLowerCase();
        } else {
            return sfds.getName();
        }


    }

    private static String getInMsgRosType(String packageName, MCTypeSymbol referencedSymbol, boolean ros2mode) {
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
            return packageName + "/" + getTargetName(structSymbol, ros2mode);
        }

        Log.error("Case not handled! MCTypeReference " + referencedSymbol);
        return null;
    }

    public static RosMsg getRosType(String packageName, MCTypeReference<? extends MCTypeSymbol> typeReference) {
        return getRosType(packageName, typeReference, false);
    }

    public static RosMsg getRosType(String packageName, MCTypeReference<? extends MCTypeSymbol> typeReference, boolean ros2mode) {
        MCTypeSymbol type = typeReference.getReferencedSymbol();
        String typeName = type.getName();
        String ros2extra = ros2mode ? "msg/" : "";

        if (typeName.equals("Q")) {
            RosMsg tmpMsg = new RosMsg("std_msgs/" + ros2extra + "Float64");
            tmpMsg.addField(new RosField("data", new RosType("float64")));
            return tmpMsg;
        } else if (typeName.equals("Z")) {
            RosMsg tmpMsg = new RosMsg("std_msgs/" + ros2extra + "Int32");
            tmpMsg.addField(new RosField("data", new RosType("int32")));
            return tmpMsg;
        } else if (typeName.equals("B")) {
            RosMsg tmpMsg = new RosMsg("std_msgs/" + ros2extra + "Bool");
            tmpMsg.addField(new RosField("data", new RosType("bool")));
            return tmpMsg;
        } else if (typeName.equals("CommonMatrixType") && type.isKindOf(MCASTTypeSymbol.KIND)) {
            ASTCommonMatrixType matrixType = (ASTCommonMatrixType) ((MCASTTypeSymbol) type).getAstType();
            String tmpMsgName = "";
            String tmpTypeName = "";
            if (matrixType.getElementType().isRational()) {
                tmpMsgName = "std_msgs/" + ros2extra + "Float64MultiArray";
                tmpTypeName = "float64";
            } else if (matrixType.getElementType().isWholeNumber()) {
                tmpMsgName = "std_msgs/" + ros2extra + "Int32MultiArray";
                tmpTypeName = "int32";
            } else if (matrixType.getElementType().isBoolean()) {
                tmpMsgName = "std_msgs/" + ros2extra + "ByteMultiArray";
                tmpTypeName = "byte";
            } else {
                Log.error("Matrix type not supported: " + matrixType);
            }
            //TODO: refactor
            return getMultMatrixRosMsg(tmpMsgName, tmpTypeName);
        }

        if (type instanceof StructSymbol) {
            StructSymbol structSymbol = (StructSymbol) type;
            return createMsgForStruct(packageName, structSymbol, ros2mode);
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

    private static String getTargetName(StructSymbol structSymbol, boolean ros2mode) {
        if (ros2mode) {
            return Arrays.stream(structSymbol.getFullName()
                    .split("\\."))
                    .map(fn -> fn.substring(0, 1).toUpperCase() + fn.substring(1))
                    .collect(Collectors.joining());
        } else {
            return structSymbol.getFullName().replace(".", "_");
        }
    }

    private static String getFullTargetName(String packageName, StructSymbol structSymbol, boolean ros2mode) {
        return ros2mode ? packageName + "/msg/" + getTargetName(structSymbol, ros2mode) : packageName + "/" + getTargetName(structSymbol, ros2mode);
    }

    public List<FileContent> generateProjectStrings(List<MCTypeReference<? extends MCTypeSymbol>> typeReferences) {
        ArrayList<FileContent> res = new ArrayList<>();

        // .msg files
        for (MCTypeReference<? extends MCTypeSymbol> tr : typeReferences) {
            for (FileContent fc : generateStrings(tr)) {
                fc.setFileName("msg/" + fc.getFileName());
                res.add(fc);
            }
        }

        if (!ros2mode) {
            res.add(getRosCMakeLists(res));
        } else {
            res.add(getRos2CMakeLists(res));
            res.add(getRos2PackageXml());
        }


        return res;
    }

    private FileContent getRos2CMakeLists(ArrayList<FileContent> res) {
        return new FileContent("CMakeLists.txt", RosMsgTemplates.generateRos2CMakeLists(new CMakeListsViewModel(res)));
    }

    private FileContent getRos2PackageXml() {
        return new FileContent("package.xml", RosMsgTemplates.generateRos2Package());
    }

    private FileContent getRosCMakeLists(ArrayList<FileContent> res) {
        return new FileContent("CMakeLists.txt", RosMsgTemplates.generateRosCMakeLists(new CMakeListsViewModel(res)));
    }

    public List<File> generateProject(List<MCTypeReference<? extends MCTypeSymbol>> typeReferences) throws IOException {
        List<File> res = new ArrayList<>();
        List<FileContent> fileContents = generateProjectStrings(typeReferences);
        for (FileContent fileContent : fileContents) {
            File file = new File(path + "/" + fileContent.getFileName());
            FileUtils.write(file, fileContent.getFileContent());
            res.add(file);
        }
        return res;
    }

    public List<File> generateProject(EMAComponentInstanceSymbol component) throws IOException {
        Stream<EMAPortInstanceSymbol> p = component.getPortInstanceList().stream();
        Stream<EMAPortInstanceSymbol> subp = component.getSubComponents().stream().flatMap(sc -> sc.getPortInstanceList().stream());
        List<MCTypeReference<? extends MCTypeSymbol>> typeReferences = Stream.concat(p, subp)
                .map(EMAPortInstanceSymbol::getTypeReference)
                .filter(SymbolReference::existsReferencedSymbol)
                .filter(mcTypeReference -> mcTypeReference.getReferencedSymbol() instanceof StructSymbol)
                .collect(Collectors.toList());

        return generateProject(typeReferences);
    }
}
