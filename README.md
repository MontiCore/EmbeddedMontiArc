<p align="center">
    <img src="targets/standalone/doc/images/elysium.png"/>
    <img src="targets/standalone/doc/images/quote.png" width="718"/>
</p>
<p align="center">
    <a href="https://github.com/theia-ide/theia">
        <img src="https://img.shields.io/badge/Theia_Version-0.3.15-blue.svg?longCache=true&style=flat-square"/>
    </a>
</p>

## Table of Contents
* [**Description**](#description)
* [**Getting Started**](/doc/GettingStarted.md)
* [**Standalone**](/targets/standalone)
* [**EmbeddedMontiArcStudio**](/targets/emastudio)
* [**Development Team**](/doc/DevelopmentTeam.md)
* [**License**](#license)

## Description
Nowadays, one of the most commonly used type of languages besides natural ones are programming
languages. Programming Languages act as instrument for the instruction of an information
system. Comparable to the dialects of natural languages, programming languages also come in
different flavors which range from imperative to object-oriented. Despite the sheer amount of
programming languages which have been developed over the course of the last decades, most of
them were not used in a single area of application. For this reason, programming languages are
often also categorized under *General Purpose Languages* (GPLs).

With the increasing prominence of GPLs due to the digitalization of our everyday life,
developers made a few observations. Designing a business system used to be a cooperation
between software developers and domain experts where the former would bring the necessary
technical expertise whereas the latter would bring the necessary knowledge about the domain
and would support the former in their task of the system's creation. The reason for this
strict separation of the two worlds is due to the fact that neither know enough about the
other's area of expertise to take over the other's part. Nonetheless, a deeper involvement of
the domain experts would dramatically increase the system's quality. For this reason, a
different kind of languages have made their appearance known under the name of
*Domain-Specific Languages* (DSLs).

DSLs are mostly constituted of a specific domain's vocabulary and are therefore often easier
to understand and to learn for a domain expert which enables a deeper involvement in the
development process. The models of a DSL can then be further processed and translated into
other languages. Therefore, there is an abstraction layer between the domain and the technical
aspects of the system.

**Elysium** is a Multi-Target IDE. One of its targets is an Online IDE whose primary focus lies
on the _client-side focused_ demonstration of and development on textual DSLs created with the
language workbench [**MontiCore**](http://www.monticore.de). The other is an Offline IDE which
acts as interface between users and projects of
[**EmbeddedMontiArc**](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc).

In order to achieve this, Elysium uses a different project as its foundation called
[**Theia**](http://www.theia-ide.org). As of this writing, Theia is the only IDE which is both
capable of running as fully fledged Online IDE on a server and as completely autonomous Offline
IDE on a PC. This is made possible due to its well-designed architecture which makes use of
state-of-the-art _Inversion of Control_ (IoC) and _Dependency Injection_ (DI) principles. Theia
itself has been developed using [**TypeScript**](https://www.typescriptlang.org) as programming
language, which is a language whose generated JavaScript files are capable of both being
executed in an online environment provided by [**Node.js**](https://nodejs.org) as server and a
browser as client component as well as in an offline environment provided by
[**Electron**](https://electronjs.org). Theia's architecture is fueled by
[**InversifyJS**](http://inversify.io), a powerful IoC framework for TypeScript and/or
JavaScript.

## License
Copyright (C) 2018 SE RWTH.

A concrete license is to be discussed.