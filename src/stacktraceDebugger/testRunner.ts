import { spawnSync } from 'child_process';
import { createHash } from 'crypto';
import { RuntimeLogger } from "./RuntimeLogger";
import { window } from 'vscode';
import { existsSync } from 'fs';

export class EMATestRunner{
	private targetBasePath:string;
	private generatorJarPath:string;
	private modelBasePath:string;
	private logger:RuntimeLogger;

	public getLogger(): RuntimeLogger {
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
     * Getter generatorJarPath
     * @return {string}
     */
	public getGeneratorJarPath(): string {
		return this.generatorJarPath;
	}

    /**
     * Getter modelBasePath
     * @return {string}
     */
	public getModelBasePath(): string {
		return this.modelBasePath;
	}

	constructor(targetBasePath: string, generatorJarPath: string, modelBasePath: string, logger: RuntimeLogger) {
		this.targetBasePath = targetBasePath;
		this.generatorJarPath = generatorJarPath;
		this.modelBasePath = modelBasePath;
		this.logger = logger;
	}

	public getTargetDir(componentName: string){
		let hash = createHash('md5').update(this.modelBasePath).update(componentName).digest("hex").substr(0, 6);
		console.log("Hash for '" + componentName + "':" + hash);
		return this.targetBasePath + "/" + hash;
	}

	public executeTests(componentName: string):boolean {
		this.logger.log("Generating...");
		let returnCode = this.generateTests(componentName);
		if(returnCode == 0){
			this.logger.log("Building and running tests...");
			this.buildAndRunTests(componentName);
			return true;
		}else{
			this.logger.log("The generator returned a non-zero return code. Aborting!");
			return false;
		}
	}

	public generateTests(componentName: string):number {
		const targetDir = this.getTargetDir(componentName);
		this.execCommand("mkdir", [this.targetBasePath], process.cwd(), false);
		this.execCommand("mkdir", [targetDir], process.cwd(), false);
		if(existsSync(this.generatorJarPath)){
			return this.execCommand("java", ["-jar", this.generatorJarPath, "--models-dir=" + this.modelBasePath, "-o=" + targetDir, "--root-model=" + componentName, "--flag-generate-tests", "--flag-use-armadillo-backend", "--flag-use-exec-logging", "--flag-use-exec-logging", "--flag-use-cmake"], targetDir, true, true);
		}else{
			this.getLogger().log("Can not find generator jar: " + this.generatorJarPath);
			return 1;
		}
	}

	public buildAndRunTests(componentName: string) {
		const targetDir = this.getTargetDir(componentName);
		this.execCommand("mkdir", [targetDir + "/build"], process.cwd(), false);
		this.execCommand("cmake", ["-B" + targetDir + "/build", "-H" + targetDir], process.cwd());
		this.execCommand("cmake", ["--build", targetDir + "/build"], process.cwd());
	}

	public execCommand(command: string, args: string[], cwd, warn:boolean = true, popup:boolean = false): number {
		console.log("EXEC COMMAND!");
		const child = spawnSync(command, args,
			{
				cwd: cwd,
				env: process.env,
				stdio: 'pipe',
				encoding: 'utf-8'
			});
		console.log("Command: " + command + " " + args.join(" "));
		if (child.status != 0) {
			const output = child.stdout.toString();
			const errOutput = child.stderr.toString();
			if (warn){
				this.logger.log("Command: " + command + " " + args.join(" "));
				this.logger.log(output);
				this.logger.log("finished with error:\n" + errOutput);
			}
			if(popup && errOutput.length > 0){
				let firstError = errOutput.split("\n")[0];
				window.showErrorMessage(firstError);
			}
		}
		return child.status;
	}
}