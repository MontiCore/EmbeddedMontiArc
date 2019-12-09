/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommonTemplateValidator } from "@embeddedmontiarc/sol-runtime-templates/lib/common/template-validator";
import { injectable } from "inversify";

export interface TestTemplateOptions {
    package: string;
    componentName: string;
    ports: {
        direction: string;
        type: string;
        name: string;
    }[];
    instances: {
        type: string;
        name: string;
    }[];
    connectors: {
        source: string;
        targets: {
            target: string;
        }[];
    }[];
}

export interface TestTemplateErrors {
    package: string | undefined;
    componentName: string | undefined;
    ports: {
        direction: string | undefined;
        type: string | undefined;
        name: string | undefined;
    }[];
    instances: {
        type: string | undefined;
        name: string | undefined;
    }[];
    connectors: {
        source: string | undefined;
        targets: {
            target: string | undefined;
        }[];
    }[];
}

@injectable()
export class TestTemplateValidator extends CommonTemplateValidator<TestTemplateOptions, TestTemplateErrors> {
    public constructor() {
        super("ema_component");
    }

    public async validate(options: TestTemplateOptions): Promise<TestTemplateErrors> {
        options = options || {};

        return {
            package: await this.validatePackage(options.package, options),
            componentName: await this.validateComponentName(options.componentName, options),
            ports: await this.validatePorts(options.ports, options),
            instances: await this.validateInstances(options.instances, options),
            connectors: await this.validateConnectors(options.connectors, options)
        };
    }

    protected async validatePackage(pack: string, options: TestTemplateOptions): Promise<string | undefined> {
        return Promise.resolve(undefined);
    }

    protected async validateComponentName(componentName: string, options: TestTemplateOptions): Promise<string | undefined> {
        return Promise.resolve(undefined);
    }

    protected async validatePorts(ports: { direction: string; type: string; name: string; }[], options: TestTemplateOptions): Promise<{ direction: string | undefined; type: string | undefined; name: string | undefined; }[]> {
        ports = ports || [];

        return Promise.all(ports.map(async port => {
            const errors = await Promise.all([
                this.validatePortsDirection(port.direction, options),
                this.validatePortsType(port.type, options),
                this.validatePortsName(port.name, options)
            ]);

            return { direction: errors[0], type: errors[1], name: errors[2] };
        }));
    }

    protected async validatePortsDirection(direction: string, options: TestTemplateOptions): Promise<string | undefined> {
        return Promise.resolve(undefined);
    }

    protected async validatePortsType(type: string, options: TestTemplateOptions): Promise<string | undefined> {
        return Promise.resolve(undefined);
    }

    protected async validatePortsName(name: string, options: TestTemplateOptions): Promise<string | undefined> {
        return Promise.resolve(undefined);
    }

    protected async validateInstances(instances: { type: string; name: string; }[], options: TestTemplateOptions): Promise<{ type: string | undefined; name: string | undefined; }[]> {
        instances = instances || [];

        return Promise.all(instances.map(async instance => {
            const errors = await Promise.all([
                this.validateInstancesType(instance.type, options),
                this.validateInstancesName(instance.name, options)
            ]);

            return { type: errors[0], name: errors[1] };
        }));
    }

    protected async validateInstancesType(type: string, options: TestTemplateOptions): Promise<string | undefined> {
        return Promise.resolve(undefined);
    }

    protected async validateInstancesName(name: string, options: TestTemplateOptions): Promise<string | undefined> {
        return Promise.resolve(undefined);
    }

    protected async validateConnectors(connectors: { source: string; targets: { target: string }[] }[], options: TestTemplateOptions): Promise<{ source: string | undefined; targets: { target: string | undefined; }[] }[]> {
        connectors = connectors || [];

        return Promise.all(connectors.map(async connector => {
            const errors = await Promise.all([
                this.validateConnectorsSource(connector.source, options),
                this.validateConnectorsTargets(connector.targets, options)
            ]);

            return { source: errors[0], targets: errors[1]! };
        }));
    }

    protected async validateConnectorsSource(source: string, options: TestTemplateOptions): Promise<string | undefined> {
        return Promise.resolve(undefined);
    }

    protected async validateConnectorsTargets(targets: { target: string }[], options: TestTemplateOptions): Promise<{ target: string | undefined }[]> {
        targets = targets || [];

        return Promise.all(targets.map(async target => {
            const errors = await Promise.all([
                this.validateConnectorsTargetsTarget(target.target, options)
            ]);

            return { target: errors[0] };
        }));
    }

    protected async validateConnectorsTargetsTarget(target: string, options: TestTemplateOptions): Promise<string | undefined> {
        return Promise.resolve(undefined);
    }
}
