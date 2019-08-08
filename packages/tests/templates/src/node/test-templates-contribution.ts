/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
import {
    TemplatesContribution,
    TemplatesRegistry
} from "@embeddedmontiarc/sol-runtime-templates/src/node/templates-contribution";
import { injectable } from "inversify";
import * as path from "path";

@injectable()
export class TestTemplatesContribution implements TemplatesContribution {
    public registerTemplates(registry: TemplatesRegistry): void {
        registry.registerTemplate({
            id: "ema_component",
            extension: ".ema",
            path: path.resolve(__dirname, "..", "..", "templates", "EmbeddedMontiArc.njk"),
            label: "EmbeddedMontiArc Component",
            elements: [{
                type: "string",
                variable: "package",
                props: {
                    label: "Package",
                    required: true
                }
            }, {
                type: "string",
                variable: "componentName",
                props: {
                    label: "Component Name",
                    required: true
                }
            }, {
                type: "list",
                variable: "ports",
                props: {
                    label: "Add Port",
                    elements: [{
                        type: "string",
                        variable: "direction",
                        props: {
                            label: "Direction",
                            required: true
                        }
                    }, {
                        type: "string",
                        variable: "type",
                        props: {
                            label: "Type",
                            required: true
                        }
                    }, {
                        type: "string",
                        variable: "name",
                        props: {
                            label: "Name",
                            required: true
                        }
                    }]
                }
            }, {
                type: "list",
                variable: "instances",
                props: {
                    label: "Add Component Instance",
                    elements: [{
                        type: "string",
                        variable: "type",
                        props: {
                            label: "Type",
                            required: true
                        }
                    }, {
                        type: "string",
                        variable: "name",
                        props: {
                            label: "Name",
                            required: true
                        }
                    }]
                }
            }, {
                type: "list",
                variable: "connectors",
                props: {
                    label: "Add Connector",
                    elements: [{
                        type: "string",
                        variable: "source",
                        props: {
                            label: "Source",
                            required: true
                        }
                    }, {
                        type: "list",
                        variable: "targets",
                        props: {
                            label: "Add Target",
                            elements: [{
                                type: "string",
                                variable: "target",
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
