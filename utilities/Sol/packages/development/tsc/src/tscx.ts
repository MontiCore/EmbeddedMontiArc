/*
 * (c) https://github.com/MontiCore/monticore
 */
import * as gulp from "gulp";
import * as ts from "gulp-typescript";
import * as minimist from "minimist";
import * as rename from "gulp-rename";
import * as stripDirs from "strip-dirs";

const argv = minimist(process.argv.slice(2));
const tsProject = ts.createProject(argv.p || argv.project || "tsconfig.json");
const shouldWatch = argv.w || argv.watch || false;
const shouldFlatten = argv.flatten || false;

tsProject.options.rootDir = ".";

const compilerOptions = tsProject.options;
const rootDirs = compilerOptions.rootDirs || [];
const outDir = compilerOptions.outDir || "lib";

gulp.task("compile", function() {
    const tsResult = tsProject.src().pipe(tsProject());
    const dest = gulp.dest(outDir);
    const flatten = rename(path => path.dirname = stripDirs(path.dirname!, 1));

    if (shouldFlatten && rootDirs.length > 1) {
        tsResult.js.pipe(flatten).pipe(dest);
        tsResult.dts.pipe(flatten).pipe(dest);
    } else {
        tsResult.js.pipe(dest);
        tsResult.dts.pipe(dest);
    }
});

gulp.task("watch", function() {
    gulp.watch("src/**/*.ts", gulp.task("compile"));
});

if (shouldWatch) gulp.task("watch")(() => console.log("DONE"));
else gulp.series("compile")(() => console.log("DONE"));
