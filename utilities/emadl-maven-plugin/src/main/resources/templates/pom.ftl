<?xml version="1.0" encoding="UTF-8"?>
<project xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 https://maven.apache.org/xsd/maven-4.0.0.xsd" xmlns="http://maven.apache.org/POM/4.0.0"
         xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
    <modelVersion>4.0.0</modelVersion>
    <groupId>${storage.groupId}</groupId>
    <artifactId>${storage.artifactId}</artifactId>
    <version>${storage.version}</version>

    <#if repository??>
    <repositories>
        <repository>
            <id>${repository.id}</id>
            <url>${repository.url}</url>
        </repository>
    </repositories>
    </#if>

    <dependencies>
    <#list dependencies as dep>
        <dependency>
            <groupId>${dep.groupId}</groupId>
            <artifactId>${dep.artifactId}</artifactId>
            <version>${dep.version}</version>
            <classifier>dataset</classifier>
        </dependency>
    </#list>
    </dependencies>
</project>