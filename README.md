# Sixth Sense
- Code is written in java 11, but should work with 8+
- src/ contains all source code
- jars/ contain all libraries bundled as jars
    - processing is used as a library

# Compilation (on linux)
- Open a terminal with current directory as the one containing this file
- Use `javac -cp "jars/*" -d build/ $(find -name "*.java")` to compile and put all output class files under build/
- Use `java -cp "build/:jars/*" <package>.<to>.<path>.<class>` to run any simulation
