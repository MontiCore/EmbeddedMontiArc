/*
 * (c) https://github.com/MontiCore/monticore
 */
import { FileUri } from "@theia/core/lib/node";
import { inject, injectable } from "inversify";
import { StaticServer } from "../common";
import { StaticService } from "./static-service";

@injectable()
export class StaticServerImpl implements StaticServer {
    @inject(StaticService) protected readonly service: StaticService;

    public async addStaticPath(uri: string): Promise<number> {
        return this.service.addStaticPath(FileUri.fsPath(uri));
    }

    public async removeStaticPath(id: number): Promise<void> {
        this.service.removeStaticPath(id);
    }
}
