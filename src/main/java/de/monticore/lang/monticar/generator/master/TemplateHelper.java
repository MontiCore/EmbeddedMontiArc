package de.monticore.lang.monticar.generator.master;

public class TemplateHelper {
    static String coordinatorTemplate =
            "#include <iostream>\n" +
                    "#include <thread>\n" +
                    "#include <chrono>\n" +
                    "#include <atomic>\n" +
                    "#include <list>\n" +
                    "#include \"IAdapter.h\"\n" +
                    "\n" +
                    "${includes}" +
                    "\n" +
                    "using namespace std;\n" +
                    "using namespace chrono;\n" +
                    "\n" +
                    "void startMiddleware(IAdapter& adapter,${compName}& comp,atomic<bool>& done){\n" +
                    "  adapter.init(&comp);\n" +
                    "  done = true;\n" +
                    "}\n" +
                    "\n" +
                    "int main() \n" +
                    "{\n" +
                    "  atomic<bool> done(false);\n" +
                    "  ${compName} comp;\n" +
                    "  comp.init();\n" +
                    "\n" +
                    "  list<IAdapter*> adapters;\n" +
                    "${addAdapters}" +
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
    static String iAdapterTemplate = "#pragma once\n" +
            "#include \"${compName}.h\"\n" +
            "\n" +
            "class IAdapter{\n" +
            "\tpublic:\n" +
            "\t\tvirtual ~IAdapter(){}\n" +
            "\t\tvirtual void init(${compName}* comp) = 0;\n" +
            "\t\tvirtual void tick() = 0;\n" +
            "};";
    static String cmakeListsTemplate =
            "cmake_minimum_required(VERSION 3.5)\n" +
                    "project (Coordinator_${compName} CXX)\n" +
                    "\n" +
                    "set (CMAKE_CXX_STANDARD 11)\n" +
                    "\n" +
                    "add_executable(Coordinator_${compName} Coordinator_${compName}.cpp)\n" +
                    "set_target_properties(Coordinator_${compName} PROPERTIES LINKER_LANGUAGE CXX)\n" +
                    "target_link_libraries(Coordinator_${compName} ${targets})\n" +
                    "target_include_directories(Coordinator_${compName} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})\n" +
                    "\n" +
                    "export(TARGETS Coordinator_${compName} FILE Coordinator_${compName}.cmake)";
}
