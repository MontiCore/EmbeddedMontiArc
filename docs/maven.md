<!-- (c) https://github.com/MontiCore/monticore -->
Prev: [Installing and using the Basic Simulator](docs/usage.md)    |    Next: [Scripts tutorial](docs/scripts.md)

---


# Maven Project tutorial

*Comming Soon*

> - Git recap
>   - Branches, local repos, commits, push/pull.
>   - Merge requests
> - Maven project: pom file and project structure
>   - Pom file: 
>     - describes the project name and version
>     - dependencies
>     - importing into maven
>   - Project structure:
>     - sources under src/main or src/test -> used by maven for the resulting jar / for the tests.
>     - java or resources folder
>     - docs / scripts / utility folders
>     - target folder: contains all the results of the build, including the jar.
>     - Packaging: try to use consistent package structure.
> - Maven dependency system: nexus, local repository and versioning
>   - Nexus: from the SE chair, contains all the versions from the master branch. Linked to in settings.xml.
>   - Local repository: (.m2/repository under windows) contains the dependencies from the nexus + the projects compiled using the "install" target: you can override versions/...
>   - Versioning: use next version with -SNAPSHOT when developing in a branch. Refer to this version locally. The commit that will be merged to master must NOT contain the -SNAPSHOT.
> - Maven plug-ins: build targets and plug-ins
> - Gitlab CI and other Repository plug-ins
>   - CI stages, runners


---

Prev: [Installing and using the Basic Simulator](docs/usage.md)    |    Next: [Scripts tutorial](docs/scripts.md)
