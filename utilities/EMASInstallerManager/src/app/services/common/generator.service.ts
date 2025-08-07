/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";

import * as path from "path";
import * as nunjucks from "nunjucks";
import * as fs from "fs-extra";

@Injectable({ providedIn: "root" })
export class GeneratorService {
    public configure(folder: string): void {
        nunjucks.configure(`assets/templates/${folder}`, { trimBlocks: true, lstripBlocks: true });
    }

    public async generate(root: string, template: string, context: any): Promise<void> {
        const relativePath = template.replace(".njk", '');
        const name = context.name ? context.name.toLowerCase() : '';
        const destination = path.join(root, relativePath).replace("{{name}}", name);
        const content = nunjucks.render(template, { context });

        await fs.ensureFile(destination);

        return fs.writeFile(destination, content);
    }
}
