PACKAGE invalid

DOCKERFILE NoCommandChaining

RUN ["echo", "Hallo", "&&", "echo", "World"]
RUN "echo Hello && echo World"