# Sixth Sense
- Code is written in Kotlin
- JRE 8 is the best as "processing" library recommends it
- src/ contains all source code
- jars/ contain all libraries bundled as jars
    - processing is used for rendering
    - emjl is used for linear algebra 

# Compilation (on linux)
- Open a terminal with current directory as the one containing this file
- `kotlinc -classpath "$(find jars/ -name "*.jar" | paste -sd ":" -)" $(find -name "*.kt") $(find -name "*.java") -include-runtime -d sixthsense.jar`
- `javac -cp "jars/*" -d build/ $(find -name "*.java")`

# Execution
- `java -cp "build/:jars/*:jars/ejml-v0.39-libs/*:sixth-sense.jar" <package>.<classname>Kt`
- Notice the suffix Kt after class name
- For example `java -cp "build/:jars/*:jars/ejml-v0.39-libs/*:sixthsense.jar" MainKt`
