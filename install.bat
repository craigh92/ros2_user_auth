echo "setting vc vars..."
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86

echo "sourcing ros..."
call C:\dev\ros2_foxy\setup.bat

echo "building..."
colcon build

echo "sourcing workspace..."
call ./install/setup.bat