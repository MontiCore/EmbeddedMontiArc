/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Configuration } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { CommonModuleCreator } from "@embeddedmontiarc/sol-runtime-modules/lib/node";
import { injectable } from "inversify";

import * as fs from "fs-extra";
import * as path from "path";

@injectable()
export class TestModuleCreator extends CommonModuleCreator {
    public constructor() {
        super("0");
    }

    public async createModules(destination: string): Promise<Configuration[]> {
        await fs.writeFile(path.resolve(destination, "Hello World.txt"), "Hello World.txt");
        return [];
    }
}
