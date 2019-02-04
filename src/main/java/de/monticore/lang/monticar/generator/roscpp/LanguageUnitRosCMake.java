package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.TemplateHelper;
import de.monticore.lang.monticar.generator.rosmsg.RosField;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class LanguageUnitRosCMake {

    private String genDepTemplate = "add_dependencies(<name> <deps>)";

    //TODO: msg->h for ROS2
    //.msg files must start with A-Z and must not contain _
    //field names must all be lowercase


    private List<RosMsg> getAllMsgs(List<RosMsg> rosMsgs){
        List<RosMsg> result = new ArrayList<>();
        for (RosMsg rosMsg : rosMsgs) {
            result.add(rosMsg);
            result.addAll(getAllMsgs(rosMsg.getFields().stream()
                    .map(RosField::getType)
                    .filter(t -> t instanceof RosMsg)
                    .map(t -> (RosMsg) t)
                    .collect(Collectors.toList())));
        }
        return result;
    }

    public List<FileContent> generate(EMAComponentInstanceSymbol componentInstanceSymbol, List<String> additionalPackages, List<RosMsg> rosMsgs, boolean isRos2Mode) {

        List<RosMsg> structMsgs = getStructMsgs(rosMsgs);

        FileContent fileContent = new FileContent();
        fileContent.setFileName("CMakeLists.txt");

        List<String> allPackages = new ArrayList<>();
        allPackages.addAll(additionalPackages);
        if(!isRos2Mode) {
            allPackages.add("roscpp");
        }else{
            allPackages.add("rclcpp");
        }

        List<String> distinctSortedPackages = allPackages.stream()
                .distinct()
                .sorted()
                .collect(Collectors.toList());

        String compName = NameHelper.getComponentNameTargetLanguage(componentInstanceSymbol.getFullName());
        String name = NameHelper.getAdapterName(componentInstanceSymbol);
        String packages = distinctSortedPackages.stream()
                .filter(p -> !p.equals("struct_msgs"))
                .map(p -> "find_package(" + p + " REQUIRED)")
                .collect(Collectors.joining("\n"));

        String libraries = distinctSortedPackages.stream()
                .filter(p -> !p.equals("struct_msgs"))
                .map(p -> "${" + p + "_LIBRARIES}")
                .collect(Collectors.joining(" "));

        String include_dirs = distinctSortedPackages.stream()
                .filter(p -> !p.equals("struct_msgs"))
                .map(p -> "${" + p + "_INCLUDE_DIRS}")
                .collect(Collectors.joining(" "));

        String dependency = "";
//        if (distinctSortedPackages.stream().filter(pack -> pack.startsWith("struct_msgs")).count() > 0) {
//            dependency = "add_dependencies(<name> struct_msgs_generate_messages)".replace("<name>", name);
//        }


        String content = TemplateHelper.getCMakeListsTemplate()
                .replace("<name>", name)
                .replace("<compName>", compName)
                .replace("<packages>", packages)
                .replace("<libraries>", libraries)
                .replace("<include_dirs>", include_dirs)
                .replace("<dependency>", dependency);

        List<FileContent> result = new ArrayList<>();
        String genMsgString = null;
        if(!isRos2Mode) {
            genMsgString = getRosMsgGenerationString(structMsgs, name);
        }else{
            genMsgString = getRos2MsgGenerationString(name);
        }
        if(structMsgs.size() > 0){
            content = content + "\n\n" + genMsgString;
            if(isRos2Mode){
                result.add(genRos2GenerationSettings(structMsgs));
                result.add(genRos2GenerateMsgsPy());
            }
        }

        fileContent.setFileContent(content);
        result.add(fileContent);

        FileContent cppFile = new FileContent();
        cppFile.setFileName(name + ".cpp");
        cppFile.setFileContent("#include \""+ name +".h\"");
        result.add(cppFile);

        return result;
    }

    private List<RosMsg> getStructMsgs(List<RosMsg> rosMsgs) {
        List<RosMsg> struct_msgs = getAllMsgs(rosMsgs).stream()
                .filter(rosMsg -> rosMsg.getName().startsWith("struct_msgs/"))
                .collect(Collectors.toList());

        Set<String> names = new HashSet<>();

        return struct_msgs.stream()
                .filter(m -> !names.contains(m.getName()))
                .peek(m -> names.add(m.getName()))
                .collect(Collectors.toList());
    }

    private FileContent genRos2GenerateMsgsPy() {
        FileContent res = new FileContent();
        res.setFileName("generateMsgs.py");
        res.setFileContent(TemplateHelper.getGenerateMsgsPyTemplate());
        return res;
    }

    private String getRos2MsgGenerationString(String name) {
        String res = "";
        res += TemplateHelper.getRos2MsgGenTemplate();
        res = res
                .replace("<struct_replaced>", "all_structs")
                .replace("<name>",name);

        res += "\n\n";
        res += genDepTemplate
                .replace("<deps>", "gen_<name>_all_structs")
                .replace("<name>",name);

        return res;
    }

    private FileContent genRos2GenerationSettings(List<RosMsg> structMsgs){
        FileContent res = new FileContent();
        res.setFileName("rclcpp_msg_gen.json");
        String msgFiles = structMsgs.stream().map(msg -> "'<cur_dir>/" + msg.getName() + ".msg'").collect(Collectors.joining(", "));

        String content = "{\n";
        content += "\t'template_dir':'<ros_base>/share/rosidl_generator_cpp/resource/',\n";
        content += "\t'target_dependencies':[" + msgFiles +"],\n";
        content += "\t'ros_interface_files':["+ msgFiles +"],\n";
        content += "\t'package_name':'struct_msgs',\n";
        content += "\t'output_dir':'<cur_dir>/struct_msgs/'\n";
        content += "}";

        res.setFileContent(content.replace("'","\""));
        return res;
    }

    private String getRosMsgGenerationString(List<RosMsg> structMsgs, String name) {
        String genCommands = structMsgs.stream()
                .map(msg -> TemplateHelper.getMsgGenTemplate()
                        .replace("<struct_replaced>",msg.getName().replace("/","_"))
                        .replace("<struct>",msg.getName())
                        .replace("<name>",name))
                .collect(Collectors.joining("\n\n"));

        String genDeps = structMsgs.stream()
                .map(msg -> "gen_" + name + "_" + msg.getName().replace("/","_"))
                .collect(Collectors.joining(" "));

        String genDependenies = genDepTemplate
                .replace("<name>",name)
                .replace("<deps>", genDeps);

        return genCommands + "\n\n" + genDependenies;
    }

}
