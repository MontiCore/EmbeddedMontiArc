/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommonVirtualTool } from "@embeddedmontiarc/sol-runtime-artifact/lib/node/tool";
import { injectable } from "inversify";

export const JavaVirtualToolFactory = Symbol("JavaVirtualToolFactory");

@injectable()
export class JavaVirtualTool extends CommonVirtualTool {
    public constructor() {
        super("java");
    }
}
