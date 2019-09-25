/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { injectable } from "inversify";
import { ApplicationClient } from "../common";
import { Server as HTTPServer } from "http";
import { Server as HTTPSServer } from "https";

import * as EventEmitter from "eventemitter3";

/**
 * @ignore
 */
@injectable()
export class BackendExternalBridge extends EventEmitter implements BackendApplicationContribution, ApplicationClient {
    public onStart(server: HTTPServer | HTTPSServer): void {
        this.emit("ready");
    }
}
