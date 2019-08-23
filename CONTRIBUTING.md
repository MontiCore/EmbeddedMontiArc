<!-- (c) https://github.com/MontiCore/monticore -->
# Contributing

THIS DOCUMENTATION IS INVALID: 
* SEE OUR SE-RULES: https://github.com/EmbeddedMontiArc/demonstrator/issues/5
* ALSO USE THE FORMATTING OF THE SE CHAIR AND **NOT** INTELLIJ ONE: https://sselab.de/lab2/private/intwiki/se-conventions/index.php?title=Code_Style_Guidelines

Here is a PDF print of it:
[Code Style Guidelines - se-conventions.pdf](https://github.com/EmbeddedMontiArc/demonstrator/files/1857943/Code.Style.Guidelines.-.se-conventions.pdf)


## Git

### Commit Messages

As a general rule, the style and formatting of commit messages should follow the guidelines in
[How to Write a Git Commit Message](http://chris.beams.io/posts/git-commit/).

If a commit is related to an issue, use `See #{issue-number}` at the end of the commit message.
Use `Fixes #{issue-number}`, `Closes #{issue-number}`, or `Resolves #{issue-number}` only in 
*pull requests to develop*.

### Branches

Follow the [GitFlow](http://nvie.com/posts/a-successful-git-branching-model/) branching 
model when creating new branches.

## Coding Conventions

### Formatting

Follow the Google Java [Style Guide](https://google.github.io/styleguide/javaguide.html). You can
find code style configurations for Eclipse and IntelliJ [here](https://github.com/google/styleguide).

### Javadoc

Write an informative Javadoc comment at least for each public method.

## Tests

* Unit tests go in `src/test/java` and end with a `Test` suffix.
* Integration tests go in `src/integration-test/java` and end with a `IT` suffix.
* Acceptance tests go in `src/acceptance-test/java` and end with a `AT` suffix.

#### Tools
Use [JUnit 5](http://junit.org/junit5/) in combination with 
[AssertJ](http://joel-costigliola.github.io/assertj/). For property based testing, use 
[QuickTheories](https://github.com/ncredinburgh/QuickTheories). For mocking, use either 
[Mockito](http://site.mockito.org/) or hand-written test doubles.

#### Test Principle
Adhere to [F.I.R.S.T](https://pragprog.com/magazines/2012-01/unit-tests-are-first) when writing
tests.
