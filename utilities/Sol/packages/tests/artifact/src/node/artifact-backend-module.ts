/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bindToolFactory } from "@embeddedmontiarc/sol-runtime-artifact/lib/node";
import { ContainerModule } from "inversify";
import { JavaVirtualTool, JavaVirtualToolFactory } from "./java-virtual-tool";

export default new ContainerModule(bind => {
    bindToolFactory(bind, JavaVirtualToolFactory, JavaVirtualTool);
});
