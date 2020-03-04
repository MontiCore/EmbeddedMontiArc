// Write your own appender by passing a config with config.func: (logEvent, layouts) => void
// Add it by using type: '@monticore/monticore-vscode-commons'
export function configure(config: any, layouts: any) {
    return (loggingEvent:any) => {
        config.func(loggingEvent, layouts);
    };;
}