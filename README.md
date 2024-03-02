# Computer-Graphics

Do:
git clone https://github.com/codepk37/Ray-Tracing-CG-simple-renderer.git

Report a

Problem statement 1-3: mentions 3 problems,refer it for understanding problem statement and expected run command

Steps :
Go to main_rendeer/simple_renderer<1 or 2 or 3>


## Compiling (Only once)
```
mkdir build
cd build
cmake ..
```

If you are on windows, this should create a Visual Studio solution ```cs7302.sln``` in the build folder. Open it and compile. \
If you are on linux/mac, you will need to additionally run the following to compile:

```
make -j8
```

## Running (Each time for rendering image)
The path to scene config (typically named `config.json`) and the path of the output image are passed using command line arguments as follows:
```bash
./build/render <scene_path> <out_path> <additional arguments see problem statement or directly respective report>

Example:
on linux:  ./render "/home/popos/Computer_Graphics/scenes/Assignment 2/Question 1/Donuts/scene.json" "/home/popos/Computer_Graphics/test1.png" "0" 
on windows: ./render.exe "/home/popos/Computer_Graphics/scenes/Assignment 2/Question 1/Donuts/scene.json" "/home/popos/Computer_Graphics/test2.png" "0"
```
