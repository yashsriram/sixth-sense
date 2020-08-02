# sixth-sense
## description
- A simple implementation of sense, plan, act loop for a differential drive in unknown static environment.
- Given a differential drive robot with a laser scanner in an static unknown 2D environment, the goal is to
    1. Create a map of the obstacles in the environment leaving no area unexplored.
    2. Simultaneously localize and map the landmarks in the environment.
    3. Not collide with any obstacles while doing so.
## roadmap
- Problems solved until now are documented in `report.pdf`
## code
- Code is written in Kotlin with Java libraries. Kotlin and Java compilers are required.
    - `src/` contains all source code.
    - `jars/` contain all libraries bundled as jars.
        - `processing` is used as a rendering library.
        - `queasy cam` is used as a camera library.
        - `emjl` is used as a linear algebra library.
    - `data/` contains resources such as images, obj, mtl files.
- JRE 8 is the best as "processing" library recommends it
## documentation
- For most of the code, the documentation is itself.
## usage
- Open a terminal at project root (the directory containing this file).
- For compilation
    - `kotlinc -classpath "$(find jars/ -name "*.jar" | paste -sd ":" -)" $(find -name "*.kt") $(find -name "*.java") -include-runtime -d sixthsense.jar`
    - `javac -cp "jars/*" -d build/ $(find -name "*.java")`
- To run a simulation
    - `java -cp "build/:jars/*:jars/ejml-v0.39-libs/*:sixth-sense.jar" <package>.<classname>Kt`
    - Notice the suffix Kt after class name
    - For example `java -cp "build/:jars/*:jars/ejml-v0.39-libs/*:sixthsense.jar" MainKt`
- Common controls
    - `w a s d` for basic camera movements.
    - `q e` for camera up and down movements.
    - `p` for pause/play.
- Tested on Ubuntu 18.04
    - If you use a distrubution that uses rolling release cycle (like Arch) you might have to install some older version of JRE and mesa (opensource intel openGL driver) that work with processing library.
## demonstration
The following video illustrates important aspects and the system in general.

[![](http://img.youtube.com/vi/B1wlH_T2Ub0/0.jpg)](https://www.youtube.com/watch?v=B1wlH_T2Ub0)
