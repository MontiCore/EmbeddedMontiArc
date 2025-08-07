/*
 * (c) https://github.com/MontiCore/monticore
 */
import * as fs from "fs-extra";
import * as nunjucks from "nunjucks";

import TemplateError = nunjucks.lib.TemplateError;

// tslint:disable:no-any

export async function render(template: string, context?: any): Promise<string> {
    const content = await fs.readFile(template, "UTF-8");

    return new Promise((resolve, reject) => {
        const callback = (error: TemplateError, result: string) => error === null ? resolve(result) : reject(error);

        nunjucks.renderString(content, context, callback);
    });
}