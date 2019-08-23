/* (c) https://github.com/MontiCore/monticore */
import {ClientFunction, Selector, t} from "testcafe";
import {Constants} from "../../se.rwth.common.test/constants";

declare var Loader;

export class LoaderTestController {
    protected $container: Selector;
    protected $message: Selector;

    public constructor() {
        this.$container = Selector("#loader-container");
        this.$message = Selector("#loader-message");
    }

    /*
     * Methods
     */
    public async show(message: string): Promise<void> {
        return ClientFunction((message) => Loader.show(message)).apply(null, arguments);
    }

    public async hide(): Promise<void> {
        return ClientFunction(() => Loader.hide()).apply(null, arguments);
    }

    public async message(message: string): Promise<void> {
        return ClientFunction((message) => Loader.message(message)).apply(null, arguments);
    }

    /*
     * Checks
     */
    protected async checkVisibility(expectedVisibility: string, timeout: number = Constants.DEFAULT_TIMEOUT): Promise<void> {
        const currentVisibility = this.$container.getStyleProperty("display");

        return t.expect(currentVisibility).eql(expectedVisibility, "[Loader]: Visibility Violation", { timeout: timeout });
    }

    public async checkShow(message: string, timeout?: number): Promise<void[]> {
        return Promise.all([
            this.checkVisibility("block", timeout),
            this.checkMessage(message)
        ]);
    }

    public async checkHide(timeout?: number): Promise<void> {
        return this.checkVisibility("none", timeout);
    }

    public async checkMessage(message: string, timeout: number = Constants.DEFAULT_TIMEOUT): Promise<void> {
        return t.expect(this.$message.innerText).eql(message, "[Loader]: Message Violation", { timeout: timeout });
    }
}
