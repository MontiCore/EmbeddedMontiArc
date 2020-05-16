/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ToolFactory } from "@embeddedmontiarc/sol-runtime-artifact/lib/node";
import { ClusterFiddleArtifact } from "@embeddedmontiarcstudio/clusterfiddle/lib/node/de.monticore.lang.monticar.utilities.clusterfiddle/artifact";
import { ClusterFiddleLibArtifact } from "@embeddedmontiarcstudio/clusterfiddle/lib/node/de.monticore.lang.monticar.utilities.clusterfiddlelib/artifact";
import { ArmadilloArtifact } from "@embeddedmontiarcstudio/emam2cpp/lib/node/de.monticore.lang.monticar.generators.emam2cpp.armadillo/artifact";
import { GppVirtualToolFactory } from "@embeddedmontiarcstudio/emam2cpp/lib/node/de.monticore.lang.monticar.generators.emam2cpp.gpp/tool-factory";
import { JDKArtifact } from "@embeddedmontiarcstudio/emam2cpp/lib/node/de.monticore.lang.monticar.generators.emam2cpp.jdk/artifact";
import { LibOpenBLAS64Artifact } from "@embeddedmontiarcstudio/emam2cpp/lib/node/de.monticore.lang.monticar.generators.emam2cpp.libopenblas64/artifact";
import { EMAM2CppToolFactory } from "@embeddedmontiarcstudio/emam2cpp/lib/node/de.monticore.lang.monticar.generators.emam2cpp/tool-factory";
import { FileUri } from "@theia/core/lib/node";
import { inject, injectable } from "inversify";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { ClusterFiddleExecuteBackendRunnerTOP } from "./backend-runner-top";

import { ClusterFiddleExecuteOptions } from "../../../common/configurations/de.monticore.lang.monticar.embeddedmontiarcstudio.clusterfiddle.clusterfiddleexecute/protocol";

import URI from "@theia/core/lib/common/uri";

import * as path from "path";
import * as fs from "fs-extra";

@injectable()
export class ClusterFiddleExecuteBackendRunner extends ClusterFiddleExecuteBackendRunnerTOP {
    @inject(ClusterFiddleArtifact) protected readonly webApp: ClusterFiddleArtifact;
    @inject(ClusterFiddleLibArtifact) protected readonly libraries: ClusterFiddleLibArtifact;
    @inject(ArmadilloArtifact) protected readonly armadillo: ArmadilloArtifact;
    @inject(JDKArtifact) protected readonly jdk: JDKArtifact;
    @inject(LibOpenBLAS64Artifact) protected readonly openBLAS64: LibOpenBLAS64Artifact;

    @inject(EMAM2CppToolFactory) protected readonly emam2cpp: ToolFactory;
    @inject(GppVirtualToolFactory) protected readonly gpp: ToolFactory;

    protected async runCopy(uuid: string, options: ClusterFiddleExecuteOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        const outputDirectory = this.prependWorkspaceRoot(context, options.outputDirectory);
        const webAppTarget = path.resolve(outputDirectory, "clusterfiddle");
        const armadilloTarget = path.resolve(outputDirectory, "native/armadillo");
        const jdkTarget = path.resolve(outputDirectory, "native/jdk");
        const libOpenBLASTarget = path.resolve(outputDirectory, "native/lib/libopenblas64.a");

        await fs.emptyDir(webAppTarget);
        await fs.emptyDir(path.resolve(outputDirectory, "native"));
        await fs.copy(this.webApp.path, webAppTarget);
        await fs.copy(this.armadillo.path, armadilloTarget);
        await fs.copy(this.jdk.path, jdkTarget);
        await fs.copy(this.openBLAS64.path, libOpenBLASTarget);
    }

    protected async runGenerate(uuid: string, options: ClusterFiddleExecuteOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        const modelsDirectory = this.prependWorkspaceRoot(context, options.modelsDirectory);
        const outputDirectory = this.prependWorkspaceRoot(context, `${options.outputDirectory}/generated-sources`);

        const process = this.emam2cpp(`
            "--models-dir=${modelsDirectory}"
            "--root-model=${options.rootModel}"
            "--output-dir=${outputDirectory}"
            --flag-use-armadillo-backend`,
        uuid);

        await this.waitForTermination(process, token);
    }

    protected async runCompile(uuid: string, options: ClusterFiddleExecuteOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        const outputDirectory = this.prependWorkspaceRoot(context, options.outputDirectory);
        const generatedSources = path.resolve(outputDirectory, "generated-sources");
        const native = path.resolve(outputDirectory, "native");
        const bin = path.resolve(outputDirectory, "clusterfiddle/resources/bin");

        await fs.ensureDir(bin);
        await fs.emptyDir(bin);
        await fs.copy(this.libraries.path, generatedSources, { overwrite: true });

        const process = this.gpp(`
            -fPIC
            -std=c++11
            -pthread
            "-I${native}/jdk"
            "-I${native}/jdk/linux"
            "-I${native}/armadillo/include"
            "-L${native}/lib"
            -o "${bin}/Clusterer"
            "${generatedSources}/mainClusterer.cpp"
            -DARMA_DONT_USE_WRAPPER -lopenblas -lm -lX11`,
        uuid);

        await this.waitForTermination(process, token);

        console.log("COMPILE");
        await new Promise(resolve => setTimeout(resolve, 10000));
    }

    protected prependWorkspaceRoot(context: OptionsContext, relativePath: string): string {
        const workspace = context.workspace && new URI(context.workspace);
        const absolutePath = workspace && FileUri.fsPath(workspace.resolve(relativePath));

        if (absolutePath) return absolutePath;
        else throw new Error("Could not derive absolute path.");
    }
}
