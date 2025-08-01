/* (c) https://github.com/MontiCore/monticore */
import { spawnSync } from 'child_process';
import { createHash } from 'crypto';
import { RuntimeLogger } from "./RuntimeLogger";
import { window } from 'vscode';
import { existsSync, mkdirSync } from 'fs';
import * as log4js from 'log4js';
import { join } from 'path';


export class EMATestRunner {
	private targetBasePath: string;
	private mavenPomPath: string;
	private modelBasePath: string;
	private logger: RuntimeLogger;

	public getDebugConsoleLogger(): RuntimeLogger {
		return this.logger;
	}

    /**
     * Getter targetBasePath
     * @return {string}
     */
	public getTargetBasePath(): string {
		return this.targetBasePath;
	}

    /**
     * Getter mavenPomPath
     * @return {string}
     */
	public getMavenPomPath(): string {
		return this.normalizePath(this.mavenPomPath);
	}

    /**
     * Getter modelBasePath
     * @return {string}
     */
	public getModelBasePath(): string {
		return this.normalizePath(this.modelBasePath);
	}

	constructor(targetBasePath: string, mavenPomPath: string, modelBasePath: string, logger: RuntimeLogger) {
		this.targetBasePath = this.normalizePath(targetBasePath);
		this.mavenPomPath = this.normalizePath(mavenPomPath);
		this.modelBasePath = this.normalizePath(modelBasePath);
		this.logger = logger;
	}

	public getTargetDir(componentName: string) {
		log4js.getLogger().trace("getTargetDir");
		let hash = createHash('md5').update(this.modelBasePath).update(componentName).digest("hex").substr(0, 6);
		log4js.getLogger().debug("Hash for '" + componentName + "':" + hash);
		return this.normalizePath(this.targetBasePath + "/" + hash);
	}

	public executeTests(componentName: string): boolean {
		log4js.getLogger().trace("executeTests");
		this.logger.log("Generating...");
		let returnCode = this.generateTests(componentName);
		if (returnCode == 0) {
			this.logger.log("Building and running tests...");
			return this.buildAndRunTests(componentName);
		} else {
			this.logger.log("The generator returned a non-zero return code. Aborting!");
			return false;
		}
	}

	public normalizePath(path: string): string {
		log4js.getLogger().trace("normalizePath");
		if (process.platform === "win32") {
			return path.replace("/", "\\");
		}
		return path.replace("\\", "/");
	}

	public generateTests(componentName: string): number {
		log4js.getLogger().trace("generateTests");
		const targetDir = this.getTargetDir(componentName);
		log4js.getLogger().trace("mkdir " + this.targetBasePath);
		if(!existsSync(this.targetBasePath)){
			mkdirSync(this.targetBasePath);
		}
		log4js.getLogger().trace("mkdir " + targetDir);
		if(!existsSync(targetDir)){
			mkdirSync(targetDir);
		}
		if (existsSync(this.mavenPomPath)) {
			return this.execCommand("mvn",
				[
					"exec:java",
					`-Dexec.args="--models-dir ${this.modelBasePath} -o ${targetDir} --root-model ${componentName} --flag-generate-tests --flag-use-armadillo-backend --flag-use-exec-logging --flag-use-exec-logging --flag-generate-cmake"`,
					"-s",
					"settings.xml"
				],
				this.mavenPomPath, true, true);
		} else {
			this.getDebugConsoleLogger().log("Can not find generator jar: " + this.mavenPomPath);
			return 1;
		}
	}

	public buildAndRunTests(componentName: string): boolean {
		log4js.getLogger().trace("buildAndRunTests");
		const targetDir = this.getTargetDir(componentName);
		const buildDir = join(targetDir, "build");
		log4js.getLogger().trace(buildDir);
		if(!existsSync(buildDir)){
			mkdirSync(buildDir);
		}
		let returnCode = this.execCommand("cmake", ["-B" + buildDir, "-H" + targetDir], process.cwd());
		if (returnCode == 0) {
			this.execCommand("cmake", ["--build", buildDir], process.cwd());
			return existsSync(join(targetDir,"build", "stacktrace.log"));
		} else {
			this.getDebugConsoleLogger().log("Error configuring CMake. Aborting!");
			return false;
		}
	}

	public execCommand(command: string, args: string[], cwd, warn: boolean = true, popup: boolean = false): number {
		log4js.getLogger().trace("execCommand");

		if(!existsSync(cwd)){
			log4js.getLogger().error("Cwd does not exist: " + cwd);
		}

		const child = spawnSync(command, args,
			{
				cwd: cwd,
				env: process.env,
				stdio: 'pipe',
				encoding: 'utf-8',
				shell: true
			});
		log4js.getLogger().debug("Command: " + command + " " + args.join(" "));
		if(child.status  == null){
			return 1;
		}
		if (child.status != 0) {
			const output = child.stdout.toString();
			const errOutput = child.stderr.toString();
			log4js.getLogger().debug("exited with code " + child.status + ". Output:");
			log4js.getLogger().debug(output);
			log4js.getLogger().debug("stderr:");
			log4js.getLogger().debug(errOutput);
			if (warn) {
				this.logger.log("Command: " + command + " " + args.join(" "));
				this.logger.log(output);
				this.logger.log("finished with error:\n" + errOutput);
			}
			if (popup && errOutput.length > 0) {
				let firstError = errOutput.split("\n")[0];
				window.showErrorMessage(firstError);
			}
		}
		return child.status;
	}
}
