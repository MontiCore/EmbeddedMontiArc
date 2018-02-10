package de.monticore.lang.monticar.generator.master;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

public class MiddlewareMasterGenerator extends CMakeMasterGenerator {

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver, String targetPath) throws IOException {
        //Add dummy GeneratorImpl for the subdir
//        String subdir = "Coordinator_"+NameHelper.getComponentNameTargetLanguage(componentInstanceSymbol.getFullName()) +"/";
        String subdir = "coordinator/";
        this.add(new GeneratorImpl() {
        }, subdir);
        List<File> files = super.generate(componentInstanceSymbol, taggingResolver, targetPath);

        files.add(FileHelper.generateFile(targetPath + subdir, generateIAdapter(componentInstanceSymbol)));
        files.add(FileHelper.generateFile(targetPath + subdir, generateCoordinator(componentInstanceSymbol, files)));
        files.add(FileHelper.generateFile(targetPath + subdir, generateCMakeList(componentInstanceSymbol, files)));
        return files;
    }

    private FileContent generateIAdapter(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        FileContent res = new FileContent();
        res.setFileName("IAdapter.h");
        String name = NameHelper.getComponentNameTargetLanguage(componentInstanceSymbol.getFullName());
        String iAdapterTemplate = "#pragma once\n" +
                "#include \"<name>.h\"\n" +
                "\n" +
                "class IAdapter{\n" +
                "\tpublic:\n" +
                "\t\tvirtual ~IAdapter(){}\n" +
                "\t\tvirtual void init(<name>* comp) = 0;\n" +
                "\t\tvirtual void tick() = 0;\n" +
                "};";

        res.setFileContent(iAdapterTemplate.replace("<name>", name));

        return res;
    }


    private String coordinatorTemplate =
            "#include <iostream>\n" +
                    "#include <thread>\n" +
                    "#include <chrono>\n" +
                    "#include <atomic>\n" +
                    "#include <list>\n" +
                    "#include \"IAdapter.h\"\n" +
                    "\n" +
                    "<includes>" +
                    "\n" +
                    "using namespace std;\n" +
                    "using namespace chrono;\n" +
                    "\n" +
                    "void startMiddleware(IAdapter& adapter,<name>& comp,atomic<bool>& done){\n" +
                    "  adapter.init(&comp);\n" +
                    "  done = true;\n" +
                    "}\n" +
                    "\n" +
                    "int main() \n" +
                    "{\n" +
                    "  atomic<bool> done(false);\n" +
                    "  <name> comp;\n" +
                    "  comp.init();\n" +
                    "\n" +
                    "  list<IAdapter*> adapters;\n" +
                    "<addAdapters>" +
                    "\n" +
                    "  list<thread*> threads;\n" +
                    "  for(auto a : adapters){\n" +
                    "    threads.push_back(new thread(startMiddleware,ref(*a),ref(comp),ref(done)));\n" +
                    "  }\n" +
                    "\n" +
                    "  cout << \"waiting for all middleware to start\\n\";\n" +
                    "  this_thread::sleep_for(seconds(3));\n" +
                    "  cout << \"started!\\n\";\n" +
                    "\n" +
                    "  int exeMs = 100;\n" +
                    "  time_point<system_clock> start, end;\n" +
                    "  while(!done){\n" +
                    "    start = system_clock::now();\n" +
                    "\n" +
                    "    comp.execute();\n" +
                    "    for(auto a : adapters){\n" +
                    "       (*a).tick();\n" +
                    "    }\n" +
                    "\n" +
                    "    end = system_clock::now();\n" +
                    "    int elapsedMs = duration_cast<milliseconds>(end-start).count();\n" +
                    "    int newSleep = exeMs - elapsedMs;\n" +
                    "    if(newSleep <= 0){\n" +
                    "      cout << \"Cant keep up! \"<< (-newSleep) <<\"ms late!\\n\";\n" +
                    "    }else{\n" +
                    "       this_thread::sleep_for(milliseconds(newSleep));\n" +
                    "    }\n" +
                    "  }\n" +
                    "\n" +
                    "  return 0;\n" +
                    "}";

    private FileContent generateCoordinator(ExpandedComponentInstanceSymbol componentInstanceSymbol, List<File> files) {


        String name = NameHelper.getComponentNameTargetLanguage(componentInstanceSymbol.getFullName());
        List<String> filesNames = files.stream()
                .map(File::getName)
                .collect(Collectors.toList());

        //TODO: DIRTY HACK!
        String includes = filesNames.stream()
                .filter(fn -> fn.matches(name + "\\w*\\.h"))
                .map(fn -> "#include \"" + fn + "\"")
                .collect(Collectors.joining("\n"));

        //TODO: DIRTY HACK!
        String addAdapters = filesNames.stream()
                .filter(fn -> fn.matches(name + "\\w*Wrapper\\.h"))
                .map(fn -> fn.substring(0, fn.length() - 2))
                .map(fn -> "  adapters.push_back(new " + fn + "());")
                .collect(Collectors.joining("\n"));

        String content = coordinatorTemplate
                .replace("<name>", name)
                .replace("<includes>", includes)
                .replace("<addAdapters>", addAdapters);


        FileContent res = new FileContent();
        res.setFileName("Coordinator_" + name + ".cpp");
        res.setFileContent(content);
        return res;
    }


    private String cmakeListsTemplate =
            "cmake_minimum_required(VERSION 3.5)\n" +
                    "project (Coordinator_<name> CXX)\n" +
                    "\n" +
                    "set (CMAKE_CXX_STANDARD 11)\n" +
                    "\n" +
                    "add_executable(Coordinator_<name> Coordinator_<name>.cpp)\n" +
                    "set_target_properties(Coordinator_<name> PROPERTIES LINKER_LANGUAGE CXX)\n" +
                    "target_link_libraries(Coordinator_<name> <targets>)\n" +
                    "target_include_directories(Coordinator_<name> PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})\n" +
                    "\n" +
                    "export(TARGETS Coordinator_<name> FILE Coordinator_<name>.cmake)";

    private FileContent generateCMakeList(ExpandedComponentInstanceSymbol componentInstanceSymbol, List<File> files) {
        FileContent res = new FileContent();

        String name = NameHelper.getComponentNameTargetLanguage(componentInstanceSymbol.getFullName());

        String targets = files.stream()
                .map(File::getName)
                .filter(fn -> fn.matches(name + "\\w*\\.h"))
                .map(fn -> fn.substring(0, fn.length() - 2))
                .collect(Collectors.joining(" "));

        String content = cmakeListsTemplate
                .replace("<targets>", targets)
                .replace("<name>", name);
        res.setFileName("CMakeLists.txt");
        res.setFileContent(content);
        return res;
    }

}
