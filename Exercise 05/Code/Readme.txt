1. Source: The codes without unit_test
2. Test: unit_test
3. Build: for Cmake and make, now it's empty
4. Data: The source image and parameters
5. CMakeLists: I changed something inside but only the path 
6. 1) First cd build
    2) cmake ..
    3) make
    4) cd ..
    5) ./build/unit_test 
    6) ./build/main 2700 1500 1000 ./data/0004.png ./data/0005.png ./data/0006.png
