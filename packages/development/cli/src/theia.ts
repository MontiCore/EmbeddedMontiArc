/*
 * (c) https://github.com/MontiCore/monticore
 */
import { render } from "./nunjucks";
import { ParsedArgs } from "minimist";
import { execa } from "./execa";

import * as fs from "fs-extra";
import * as path from "path";

async function theiabuild<T>(argv: T & ParsedArgs): Promise<void> {
    const content = await render(path.resolve(__dirname, "..", "src", "templates", "webpack.config.njk"));
    const outputFile = path.resolve(process.cwd(), "webpack.config++.js");
    const rest = process.argv.slice(3);
    const args = argv.hasOwnProperty("config") ? rest : [...rest, "--config", "webpack.config++.js"];

    await fs.writeFile(outputFile, content);
    await execa("theia", args);
}

async function theiaclean(): Promise<void> {
    await Promise.all([
        execa("theia", process.argv.slice(3)),
        fs.remove(path.resolve(process.cwd(), "webpack.config++.js"))
    ]);
}

async function fallback(): Promise<void> {
    await execa("theia", process.argv.slice(3));
}

export async function theia<T>(argv: T & ParsedArgs): Promise<void> {
    switch (argv._[1]) {
        case "build": return theiabuild(argv);
        case "clean": return theiaclean();
        default: return fallback();
    }
}
