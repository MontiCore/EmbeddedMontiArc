/*
 * (c) https://github.com/MontiCore/monticore
 */
import {
    TemplatesContribution,
    TemplatesRegistry
} from "@embeddedmontiarc/sol-runtime-templates/lib/node/templates-registry";
import { injectable } from "inversify";
import * as path from "path";

@injectable()
export class TestTemplateContribution implements TemplatesContribution {
    public registerTemplates(registry: TemplatesRegistry): void {
        registry.registerTemplate({
            id: "ema_component",
            extension: ".ema",
            path: path.resolve(__dirname, "..", "..", "templates", "EmbeddedMontiArc.njk"),
            label: "EmbeddedMontiArc Component",
            options: [{
                type: "de.monticore.lang.monticar.sol.option.types.String",
                name: "package",
                props: {
                    label: "Package",
                    required: true
                }
            }, {
                type: "de.monticore.lang.monticar.sol.option.types.String",
                name: "componentName",
                props: {
                    label: "Component Name",
                    required: true
                }
            }, {
                type: "de.monticore.lang.monticar.sol.option.types.List",
                name: "ports",
                props: {
                    label: "Add Port",
                    options: [{
                        type: "de.monticore.lang.monticar.sol.option.types.String",
                        name: "direction",
                        props: {
                            label: "Direction",
                            required: true
                        }
                    }, {
                        type: "de.monticore.lang.monticar.sol.option.types.String",
                        name: "type",
                        props: {
                            label: "Type",
                            required: true
                        }
                    }, {
                        type: "de.monticore.lang.monticar.sol.option.types.String",
                        name: "name",
                        props: {
                            label: "Name",
                            required: true
                        }
                    }]
                }
            }, {
                type: "de.monticore.lang.monticar.sol.option.types.List",
                name: "instances",
                props: {
                    label: "Add Component Instance",
                    options: [{
                        type: "de.monticore.lang.monticar.sol.option.types.String",
                        name: "type",
                        props: {
                            label: "Type",
                            required: true
                        }
                    }, {
                        type: "de.monticore.lang.monticar.sol.option.types.String",
                        name: "name",
                        props: {
                            label: "Name",
                            required: true
                        }
                    }]
                }
            }, {
                type: "de.monticore.lang.monticar.sol.option.types.List",
                name: "connectors",
                props: {
                    label: "Add Connector",
                    options: [{
                        type: "de.monticore.lang.monticar.sol.option.types.String",
                        name: "source",
                        props: {
                            label: "Source",
                            required: true
                        }
                    }, {
                        type: "de.monticore.lang.monticar.sol.option.types.List",
                        name: "targets",
                        props: {
                            label: "Add Target",
                            options: [{
                                type: "de.monticore.lang.monticar.sol.option.types.String",
                                name: "target",
                                props: {
                                    label: "Target",
                                    required: true
                                }
                            }]
                        }
                    }]
                }
            }]
        });
    }
}
