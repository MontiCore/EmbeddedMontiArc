/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommonValidator } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { FileSystem } from "@theia/filesystem/lib/common";
import { inject, injectable, unmanaged } from "inversify";

import URI from "@theia/core/lib/common/uri";

@injectable()
export abstract class CommonConfigurationValidator<V, E> extends CommonValidator<V, E> {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    protected constructor(@unmanaged() id: string) {
        super(id, "configuration");
    }

    protected async validateQualifiedName(qualifiedName: string, modelPath?: string, extension?: string): Promise<boolean> {
        if (modelPath && extension) return this.validateModelExistence(qualifiedName, modelPath, extension);
        else return this.validateQualifiedNameFormat(qualifiedName);
    }

    protected async validateQualifiedNameFormat(qualifiedName: string): Promise<boolean> {
        const pattern = /^([a-zA-Z_$][a-zA-Z\d_$]*\.)*[a-zA-Z_$][a-zA-Z\d_$]*$/;
        const matches = qualifiedName.match(pattern);

        return matches !== null;
    }

    protected async validateModelExistence(qualifiedName: string, modelPath: string, extension: string): Promise<boolean> {
        const qualifiedParts = qualifiedName.split('.');
        const length = qualifiedParts.length;
        const normalizedExtension = extension.startsWith('.') ? extension : `.${extension}`;

        let uriModel = new URI(modelPath);

        qualifiedParts[length - 1] = qualifiedParts[length - 1] + normalizedExtension;

        qualifiedParts.forEach(qualifiedPart => uriModel = uriModel.resolve(qualifiedPart));

        return this.fileSystem.exists(uriModel.toString());
    }
}
