PACKAGE valid

DOCKERFILE NoCommandChaining

RUN ["echo", "Hallo"]
RUN ["echo", "World"]
RUN "echo Hello"
RUN "echo World"