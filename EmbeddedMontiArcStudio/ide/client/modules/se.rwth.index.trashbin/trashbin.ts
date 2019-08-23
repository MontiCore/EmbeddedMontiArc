/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.framework/module";

export class Trashbin extends Module {
    protected mountPoints: string[];

    protected constructor() {
        super();
        this.load();
    }

    protected static instance: Trashbin;

    public static create(): Trashbin {
        return Trashbin.instance ? Trashbin.instance : (Trashbin.instance = new Trashbin);
    }


    private static readonly KEY: string = "trashbin";

    protected load(): void {
        const data = window.localStorage.getItem(Trashbin.KEY) || "[]";

        this.mountPoints = JSON.parse(data);
        this.emit("load");
    }

    protected save(): void {
        const data = JSON.stringify(this.mountPoints);

        window.localStorage.setItem(Trashbin.KEY, data);
        this.emit("save");
    }

    public contains(mountPoint: string): boolean {
        return this.mountPoints.indexOf(mountPoint) > -1;
    }

    public add(mountPoint: string): void {
        this.mountPoints.push(mountPoint);
        this.emit("add", mountPoint);
        this.save();
    }

    public remove(mountPoint: string): void {
        const index = this.mountPoints.indexOf(mountPoint);

        this.mountPoints.splice(index, 1);
        this.emit("remove", mountPoint);
        this.save();
    }

    public empty(): void {
        const onSuccess = () => {
            const mountPoint = this.mountPoints.shift();

            if(mountPoint) this.handleEmpty(mountPoint, onSuccess);
            else this.save();
        };

        onSuccess();
    }

    private handleEmpty(mountPoint: string, onSuccess: (...args) => void): void {
        const request = window.indexedDB.deleteDatabase(mountPoint);

        function onError(event) {
            console.error(event.result.target);
            this.save();
        }

        request.onsuccess = onSuccess;
        request.onerror = onError;
    }
}
