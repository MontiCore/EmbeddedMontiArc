PACKAGE EnvironmentValidatorImpl

COMPONENT DOCKERFILE NoAPTGetInstall

RUN "apt-get install -y openjdk"
RUN ["apt-get", "install", "-y", "openjdk"]