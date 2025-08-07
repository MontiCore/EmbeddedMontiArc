<#-- (c) https://github.com/MontiCore/monticore -->
void logStart(){
    std::ofstream __StacktraceFile;
    __StacktraceFile.open("stacktrace.log", std::ios_base::out | std::ios_base::app);
    __StacktraceFile << "Breakpoint reached" << std::endl;
<#list viewModel.getInstanceStack() as curSym>
    __StacktraceFile << "\tat ${curSym.getFullName()}(${curSym.getComponentType().getReferencedSymbol().getFullName()?replace(".","/")}.emam:1)" << std::endl;
</#list>

    __StacktraceFile << "#tick " << __EXECCOUNTER << std::endl;
    addVariablesToStream(__StacktraceFile, true, true);
    __StacktraceFile << "endBreakpoint" << std::endl;
    __StacktraceFile.close();
}

void logEnd(){
    std::ofstream __LogExecutionFile;
    std::ofstream __StacktraceFile;
    __LogExecutionFile.open("execution" + std::to_string(__EXECCOUNTER) + "${viewModel.getOriginalSymbol().getPackageName()}.${viewModel.getOriginalSymbol().getName()}.res");
    __StacktraceFile.open("stacktrace.log", std::ios_base::out | std::ios_base::app);

    __StacktraceFile << "Breakpoint reached" << std::endl;
<#list viewModel.getInstanceStack() as curSym>
    <#if curSym == viewModel.getOriginalSymbol()>
        <#assign line=viewModel.getLastLineOfInstance()/>
    <#else>
        <#assign line="1"/>
    </#if>
    __StacktraceFile << "\tat ${curSym.getFullName()}(${curSym.getComponentType().getReferencedSymbol().getFullName()?replace(".","/")}.emam:${line})" << std::endl;
</#list>

    __StacktraceFile << "#tick " << __EXECCOUNTER << std::endl;
    addVariablesToStream(__StacktraceFile, true, false);
    __StacktraceFile << "endBreakpoint" << std::endl;
    addVariablesToStream(__LogExecutionFile, false, false);
    __StacktraceFile.close();
    __LogExecutionFile.close();
    __EXECCOUNTER = __EXECCOUNTER + 1;
}

void addVariablesToStream(std::ofstream& stream, bool type, bool onlyIncoming){
<#list viewModel.getVariables() as var>
    <#assign type=viewModel.getTypeStringForVar(var)/>
    <#if type != "in">
    if(!onlyIncoming){
    </#if>
        stream << (type ? "${type} ${var.getVariableType().getTypeNameMontiCar()} " : "");
        stream << "${var.getNameTargetLanguageFormat()} : ";
        toFileString(stream, ${var.getNameTargetLanguageFormat()});
        stream << std::endl;
    <#if type != "in">
    }
    </#if>
</#list>
}
