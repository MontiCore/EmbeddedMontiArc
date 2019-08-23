/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.module/module";

interface Check {
    check(): Promise<void>;
}

abstract class AbstractCheck implements Check {
    public abstract async check(): Promise<void>;
}

interface CheckCollection extends Check {
    addCheck(check: Check | CheckCollection): void;
    getAll(): Check[];
}

abstract class AbstractCheckCollection extends AbstractCheck implements CheckCollection {
    protected checks: Check[];

    public constructor() {
        super();
        this.checks = [];
    }

    public addCheck(check: Check | CheckCollection): void {
        if(check instanceof AbstractCheck) this.addCheckCollection(check);
        else this.addCheckSingle(check);
    }

    private addCheckSingle(check: Check): void {
        this.checks.push(check);
    }

    private addCheckCollection(collection: CheckCollection): void {
        const checks = collection.getAll();
        const length = checks.length;

        for(let i = 0; i < length; i++) {
            this.checks.push(checks[i]);
        }
    }

    public getAll(): Check[] {
        return this.checks;
    }

    public async check(): Promise<void> {
        const checks = this.checks;
        const length = checks.length;

        for(let i = 0; i < length; i++) {
            await checks[i].check();
        }
    }
}

class MobilesCheck extends AbstractCheck {
    
}

class LocalStorageCheck extends AbstractCheck {

}

class IndexedDBCheck extends AbstractCheck {

}

class WebAssemblyCheck extends AbstractCheck {
    public async check(): Promise<void> {
        if(window.WebAssembly === undefined)
    }
}

class APIChecks extends AbstractCheckCollection {
    public constructor() {
        super();
        this.addCheck(new LocalStorageCheck);
        this.addCheck(new IndexedDBCheck);
        this.addCheck(new WebAssemblyCheck);
    }
}

class ChromeCheck extends AbstractCheck {

}

class EdgeCheck extends AbstractCheck {

}

class BrowserChecks extends AbstractCheckCollection {
    public constructor() {
        super();
        this.addCheck(new ChromeCheck);
        this.addCheck(new EdgeCheck);
    }
}

export class Checker extends Module {
    public constructor() {
        super("common.checker");
    }
}
