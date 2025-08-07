/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Process } from "@theia/process/lib/node";

export const ToolFactory = Symbol("ToolFactory");
export interface ToolFactory {
    (argstring: string, uuid: string): Process;
}
