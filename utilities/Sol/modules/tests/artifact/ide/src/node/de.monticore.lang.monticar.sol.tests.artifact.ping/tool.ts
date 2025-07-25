/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { PingToolTOP } from "./tool-top";
import { Process } from "@theia/process/lib/node";

@injectable()
export class PingTool extends PingToolTOP {
    public run(argstring: string, uuid: string): Process {
        console.log(`Running PingTool with ${argstring}`);
        return super.run(argstring, uuid);
    }
}
