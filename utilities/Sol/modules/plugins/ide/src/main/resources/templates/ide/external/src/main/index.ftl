/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("ide", "extensions")}
<#assign name = ide.getName()>
<#assign registry = ide.getRegistry().get()>
import "reflect-metadata";

import { container } from "@embeddedmontiarc/sol-external-core/lib/common/ioc";
import { Application } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { app } from "electron";

import * as Express from "express";
import * as Path from "path";

import * as debug from "electron-debug";

if (process.argv.indexOf("--debug") > -1) debug({ isEnabled: true, showDevTools: true, devToolsMode: "detach" });

async function execute() {
    const renderer = Path.resolve(app.getAppPath(), "lib", "renderer");
    const modules = await Promise.all([
        <#list extensions as extension>
        import("${extension}")<#if extension?has_next>,</#if>
        </#list>
    ]);

    modules.forEach(module => container.load(module.default));

    const application = container.get<Application>(Application);

    application.use(Express.static(renderer));
    application.setName("${name}");
    application.setImage("${registry}");
    app.on("ready", () => application.start());
}

execute().catch(error => console.error(error));
