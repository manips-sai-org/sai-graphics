# sai-graphics

This is SAI graphics library for simulation displays.
It uses [CHAI3D](https://github.com/manips-sai-org/chai3d).
You will need to download and build Chai3d before you can install sai-graphics.

## Build instructions 
```
 mkdir build
 cd build
 cmake .. && make -j8
 cd ..
```

 ## Run the examples
Go to the build/examples/one_of_the_examples folder and run the example. For example 1 :
```
cd build/examples/01-parse_world_and_robot
./01-parse_world_and_robot
```

## Documentation
The documentation can also be accessed online at the following [link](https://manips-sai-org.github.io/sai-graphics/)

It can also be generated locally with doxygen:
```
cd docs
doxygen
```

## Note on supported graphics files

SAI graphics rendering supports visuals defined by primitive shapes (box, shpere, cylinder) and the following mesh file formats:
- obj (with associated mtl files)
- 3ds
- stl (binary stl only, not ascii stl)
Files using other format can be converted using a software like [blender](https://www.blender.org).

#### Notes on converting files to obj with blender 
* When converting files to obj using blender, the associated mtl will often have a line defining the Ka value that will put all the Ka values to 1: `Ka 1.000000 1.000000 1.000000`. This will make all the material appear white as they will emit white light. You will need to replace this with `Ka 0.000000 0.000000 0.000000` in order to see the actual colors of the material definition.
* When converting files using blender, be careful to keep the same import and export conditions (for example, Y forward, Z up). Different file formats tend to have different default conventions.

## License
Currently pending licensing. PLEASE DO NOT DISTRIBUTE.
