/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommonModuleValidator } from "@embeddedmontiarc/sol-runtime-modules/lib/common/module-validator";
import { injectable } from "inversify";

import * as fs from "fs-extra";
import { FileUri } from "@theia/core/lib/node";

@injectable()
export class TestModuleValidator extends CommonModuleValidator {
    public constructor() {
        super("0");
    }

    public async validate(destination: string): Promise<string | true> {
        const contents = await fs.readdir(FileUri.fsPath(destination));

        if (contents.length > 0) return "The chosen destination is not empty.";

        return true;
    }
}
