${tc.signature("extensions")}
/*
 * (c) https://github.com/MontiCore/monticore
 */
import "reflect-metadata";

import { container } from "@embeddedmontiarc/sol-external-core/lib/common/ioc";
import { Application } from "@embeddedmontiarc/sol-external-core/lib/renderer/application";

async function execute() {
    const modules = await Promise.all([
        <#list extensions as extension>
        import("${extension}")<#if extension?has_next>,</#if>
        </#list>
    ]);

    modules.forEach(module => container.load(module.default));

    const application = container.get<Application>(Application);

    application.setLogo(require("../../build/logo.png"));
    return application.render();
}

execute().catch(error => console.error(error));
