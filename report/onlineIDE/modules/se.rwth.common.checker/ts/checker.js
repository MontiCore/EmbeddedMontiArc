var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : new P(function (resolve) { resolve(result.value); }).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
define(["require", "exports"], function (require, exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    class AbstractCheck {
    }
    class AbstractCheckCollection extends AbstractCheck {
        constructor() {
            super();
            this.checks = [];
        }
        addCheck(check) {
            if (check instanceof AbstractCheck)
                this.addCheckCollection(check);
            else
                this.addCheckSingle(check);
        }
        addCheckSingle(check) {
            this.checks.push(check);
        }
        addCheckCollection(collection) {
            const checks = collection.getAll();
            const length = checks.length;
            for (let i = 0; i < length; i++) {
                this.checks.push(checks[i]);
            }
        }
        getAll() {
            return this.checks;
        }
        check() {
            return __awaiter(this, void 0, void 0, function* () {
                const checks = this.checks;
                const length = checks.length;
                for (let i = 0; i < length; i++) {
                    yield checks[i].check();
                }
            });
        }
    }
    class MobilesCheck extends AbstractCheck {
    }
    class LocalStorageCheck extends AbstractCheck {
    }
    class IndexedDBCheck extends AbstractCheck {
    }
    class WebAssemblyCheck extends AbstractCheck {
        check() {
            return __awaiter(this, void 0, void 0, function* () {
                if (window.WebAssembly === undefined)
                    ;
            });
        }
    }
    class APIChecks extends AbstractCheckCollection {
        constructor() {
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
        constructor() {
            super();
            this.addCheck(new ChromeCheck);
            this.addCheck(new EdgeCheck);
        }
    }
    class Checker extends AbstractCheckCollection {
        constructor() {
            super();
            this.addCheck(new MobilesCheck);
            this.addCheck(new BrowserChecks);
            this.addCheck(new APIChecks);
        }
    }
    exports.Checker = Checker;
});
//# sourceMappingURL=checker.js.map