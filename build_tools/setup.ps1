$BuildType = "Release"

# Configure environment
$ErrorActionPreference = "Stop"
Set-PSDebug -Trace 1

$RootDir = (Get-Item -Path "..\").FullName
if (Test-Path Env:/GITHUB_ACTIONS) {
  $RootDir = $Env:GITHUB_WORKSPACE
}
$InstallDir = "$RootDir\install"

if (-not (Test-Path -PathType Container $InstallDir)) {
  New-Item -ItemType "directory" -Force -Path "$InstallDir"
}

### Remove the preinstalled boost library from path
if (Test-Path Env:/Boost_ROOT) {
  Remove-Item Env:/Boost_ROOT
}

### cmake put them under .\lib\pkgconfig or .\share\pkgconfig
$Env:PKG_CONFIG_PATH = "$InstallDir\lib\pkgconfig;$InstallDir\share\pkgconfig"

### Alter the path to ensure the executable files (and DLL ?) are found
$Env:Path += ";$InstallDir\bin"

# Build boost (boost numeric odeint < 1.71 does not support eigen3 > 3.2 and eigen < 3.3 build fails on windows because of a cmake error)

### Download boost
# $BoostDownloadPath = "https://dl.bintray.com/boostorg/release/1.${Env:BOOST_MINOR_VERSION}.0/source/boost_1_${Env:BOOST_MINOR_VERSION}_0.zip"
# (new-object System.Net.WebClient).DownloadFile($BoostDownloadPath, "$RootDir\boost_1_${Env:BOOST_MINOR_VERSION}_0.zip")
# Add-Type -A System.IO.Compression.FileSystem
# [IO.Compression.ZipFile]::ExtractToDirectory("$RootDir\boost_1_${Env:BOOST_MINOR_VERSION}_0.zip", "$RootDir")

### Patch \boost\python\operators.hpp (or \libs\python\include\boost\python\operators.hpp on github) to avoid conflicts with msvc
$LineNumbers = @(22, 371)
$Contents = Get-Content $RootDir\boost_1_${Env:BOOST_MINOR_VERSION}_0\libs\python\include\boost\python\operators.hpp
Set-PSDebug -Trace 0
$Contents | Foreach {$n=1}{if ($LineNumbers[0] -eq $n) {
'// Workaround msvc iso646.h
#if defined(_MSC_VER) && !defined(__clang__)
#ifndef __GCCXML__
#if defined(or)
#   pragma push_macro("or")
#   pragma push_macro("xor")
#   pragma push_macro("and")
#   undef or
#   undef xor
#   undef and
#endif
#endif
#endif'
} elseif ($LineNumbers[1] -eq $n) {
'// Workaround msvc iso646.h
#if defined(_MSC_VER) && !defined(__clang__)
#ifndef __GCCXML__
#if defined(or)
#   pragma pop_macro("or")
#   pragma pop_macro("and")
#   pragma pop_macro("xor")
#endif
#endif
#endif'
} ; $_ ; $n++} > $RootDir\boost_1_${Env:BOOST_MINOR_VERSION}_0\libs\python\include\boost\python\operators.hpp
Set-PSDebug -Trace 1

### Build the build tool b2 (build-ception !)
Set-Location -Path "$RootDir\boost_1_${Env:BOOST_MINOR_VERSION}_0"
.\bootstrap.bat --prefix="$InstallDir"

### Build and install boost
$BuildTypeB2 = $BuildType.ToLower()
if (-not (Test-Path -PathType Container $RootDir\boost_1_${Env:BOOST_MINOR_VERSION}_0\build)) {
  New-Item -ItemType "directory" -Force -Path "$RootDir\boost_1_${Env:BOOST_MINOR_VERSION}_0\build"
}
.\b2.exe --prefix="$InstallDir" --build-dir="$RootDir\boost_1_${Env:BOOST_MINOR_VERSION}_0\build" `
         --without-wave --without-contract --without-graph --without-regex `
         --without-mpi --without-coroutine --without-fiber --without-context `
         --without-timer --without-thread --without-chrono --without-date_time `
         --without-atomic --without-graph_parallel --without-type_erasure `
         --without-container --without-exception --without-locale --without-log `
         --without-program_options --without-random --without-math --without-iostreams `
         --build-type=minimal toolset=msvc-14.0 variant=$BuildTypeB2 threading=multi -q -d0 -j2 `
         architecture=x86 address-model=64 link=shared runtime-link=shared install

# Build eigen3
if (-not (Test-Path -PathType Container $RootDir\eigen3\build)) {
  New-Item -ItemType "directory" -Force -Path "$RootDir\eigen3\build"
}
Set-Location -Path $RootDir\eigen3\build
cmake -G "Visual Studio 15" -T "v140" -DCMAKE_GENERATOR_PLATFORM=x64 -DCMAKE_CXX_STANDARD=11 -DCMAKE_INSTALL_PREFIX="$InstallDir" `
                                      -DBUILD_TESTING=OFF -DEIGEN_BUILD_PKGCONFIG=ON `
                                      -DCMAKE_CXX_FLAGS="/bigobj" $RootDir\eigen3
cmake --build . --target install --config "$BuildType"

# Build eigenpy

###
if (-not (Test-Path -PathType Container $RootDir\eigenpy\build)) {
  New-Item -ItemType "directory" -Force -Path "$RootDir\eigenpy\build"
}
Set-Location -Path $RootDir\eigenpy\build
cmake -G "Visual Studio 15" -T "v140" -DCMAKE_GENERATOR_PLATFORM=x64 -DCMAKE_CXX_STANDARD=11 -DCMAKE_INSTALL_PREFIX="$InstallDir" `
                                      -DBOOST_ROOT="$InstallDir" -DBoost_USE_STATIC_LIBS=OFF `
                                      -DBoost_NO_BOOST_CMAKE=TRUE -DBoost_NO_SYSTEM_PATHS=TRUE `
                                      -DBUILD_TESTING=OFF `
                                      -DCMAKE_CXX_FLAGS="/EHsc /bigobj -DBOOST_ALL_NO_LIB -DBOOST_LIB_DIAGNOSTIC" $RootDir\eigenpy
cmake --build . --target install --config "$BuildType" --parallel 2

### Must patch line 18 of $InstallDir\lib\pkgconfig\eigenpy.pc because if the list of library includes in ill-formated on Windows.
#   The pkconfig config file is generated by the cmake submodule, which is the same for pinocchio itself.
$LineNumber = 18
$Contents = Get-Content $InstallDir\lib\pkgconfig\eigenpy.pc
Set-PSDebug -Trace 0
$Contents | Foreach {$n=1}{if ($LineNumber -eq $n) {'Libs:'} else {$_} ; $n++ } > $InstallDir\lib\pkgconfig\eigenpy.pc
Set-PSDebug -Trace 1

# Build tinyxml
if (-not (Test-Path -PathType Container $RootDir\tinyxml\build)) {
  New-Item -ItemType "directory" -Force -Path "$RootDir\tinyxml\build"
}
Set-Location -Path $RootDir\tinyxml\build
cmake -G "Visual Studio 15" -T "v140" -DCMAKE_GENERATOR_PLATFORM=x64 -DCMAKE_CXX_STANDARD=11 -DCMAKE_INSTALL_PREFIX="$InstallDir" `
                                      -DCMAKE_CXX_FLAGS="/EHsc /bigobj" $RootDir\tinyxml
cmake --build . --target install --config "$BuildType" --parallel 2

# Build console_bridge

### Must remove lines 107 and 114 of CMakefile.txt `if (NOT MSVC) ... endif()`
$LineNumbers = @(107, 114)
$Contents = Get-Content $RootDir\console_bridge\CMakeLists.txt
Set-PSDebug -Trace 0
$Contents | Foreach {$n=1}{if (-Not ($LineNumbers -Contains $n)) {$_} ; $n++} > $RootDir\console_bridge\CMakeLists.txt
Set-PSDebug -Trace 1

###
if (-not (Test-Path -PathType Container $RootDir\console_bridge\build)) {
  New-Item -ItemType "directory" -Force -Path "$RootDir\console_bridge\build"
}
Set-Location -Path $RootDir\console_bridge\build
cmake -G "Visual Studio 15" -T "v140" -DCMAKE_GENERATOR_PLATFORM=x64 -DCMAKE_CXX_STANDARD=11 -DCMAKE_INSTALL_PREFIX="$InstallDir" `
                                      -DCMAKE_CXX_FLAGS="/EHsc /bigobj" $RootDir\console_bridge
cmake --build . --target install --config "$BuildType" --parallel 2

# Build urdfdom_headers

### Must remove lines 51 and 56 of CMakefile.txt `if (NOT MSVC) ... endif()`
$LineNumbers = @(51, 56)
$Contents = Get-Content $RootDir\urdfdom_headers\CMakeLists.txt
Set-PSDebug -Trace 0
$Contents | Foreach {$n=1}{if (-Not ($LineNumbers -Contains $n)) {$_} ; $n++} > $RootDir\urdfdom_headers\CMakeLists.txt
Set-PSDebug -Trace 1

###
if (-not (Test-Path -PathType Container $RootDir\urdfdom_headers\build)) {
  New-Item -ItemType "directory" -Force -Path "$RootDir\urdfdom_headers\build"
}
Set-Location -Path $RootDir\urdfdom_headers\build
cmake -G "Visual Studio 15" -T "v140" -DCMAKE_GENERATOR_PLATFORM=x64 -DCMAKE_CXX_STANDARD=11 -DCMAKE_INSTALL_PREFIX="$InstallDir" `
                                      -DCMAKE_CXX_FLAGS="/EHsc /bigobj" $RootDir\urdfdom_headers
cmake --build . --target install --config "$BuildType" --parallel 2

# Build urdfdom

### Must remove lines 68 and 86 of CMakefile.txt `if (NOT MSVC) ... endif()`
$LineNumbers = @(78, 86)
$Contents = Get-Content $RootDir\urdfdom\CMakeLists.txt
Set-PSDebug -Trace 0
$Contents | Foreach {$n=1}{if (-Not ($LineNumbers -Contains $n)) {$_} ; $n++} > $RootDir\urdfdom\CMakeLists.txt
Set-PSDebug -Trace 1

###
if (-not (Test-Path -PathType Container $RootDir\urdfdom\build)) {
  New-Item -ItemType "directory" -Force -Path "$RootDir\urdfdom\build"
}
Set-Location -Path $RootDir\urdfdom\build
cmake -G "Visual Studio 15" -T "v140" -DCMAKE_GENERATOR_PLATFORM=x64 -DCMAKE_CXX_STANDARD=11 -DCMAKE_INSTALL_PREFIX="$InstallDir" `
                                      -DBUILD_TESTING=OFF -DTinyXML_ROOT_DIR="$InstallDir" `
                                      -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE -DBUILD_SHARED_LIBS=TRUE `
                                      -DCMAKE_CXX_FLAGS="/EHsc /bigobj -D_USE_MATH_DEFINES -DURDFDOM_EXPORTS" $RootDir\urdfdom # -DURDFDOM_STATIC or -DURDFDOM_EXPORTS ?
cmake --build . --target install --config "$BuildType" --parallel 2

# Build Pinocchio

### Must add line after 337 (387 v2.1.2 eigenpy) of cmake\python.cmake to remove disk prefix in target names and shorten its name to less than 50 chars
$LineNumber = 337
$Contents = Get-Content $RootDir\pinocchio\cmake\python.cmake
Set-PSDebug -Trace 0
$Contents | Foreach {$n=1}{if ($LineNumber -eq $n) {
'string(LENGTH ${FILE_TARGET_NAME} FILE_TARGET_LENGTH)
math(EXPR FILE_TARGET_START "${FILE_TARGET_LENGTH}-50")
if(${FILE_TARGET_START} LESS 3)
  set(FILE_TARGET_START 3)
endif()
string(SUBSTRING ${FILE_TARGET_NAME} ${FILE_TARGET_START} -1 FILE_TARGET_NAME)
'} ; $_ ; $n++ } > $RootDir\pinocchio\cmake\python.cmake
Set-PSDebug -Trace 1

### add line before 170 of \CMakeLists.txt to include directory of python lib
$LineNumber = 170
$Contents = Get-Content $RootDir\pinocchio\CMakeLists.txt
Set-PSDebug -Trace 0
$Contents | Foreach {$n=1}{if ($LineNumber -eq $n) {
'link_directories(SYSTEM "${PYTHON_LIBRARY_DIRS}")'
} ; $_ ; $n++ } > $RootDir\pinocchio\CMakeLists.txt
Set-PSDebug -Trace 1

### For some reason, the preprocessor directive `PINOCCHIO_EIGEN_PLAIN_TYPE((...))` is not properly generated. Expending it manually
#   in src\algorithm\joint-configuration.hpp and src\algorithm\joint-configuration.hxx
$FileNames = @("$RootDir\pinocchio\src\algorithm\joint-configuration.hpp", "$RootDir\pinocchio\src\algorithm\joint-configuration.hxx")
$DirectiveOrign = 'PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))'
$DirectiveAfter = 'Eigen::internal::plain_matrix_type< typename pinocchio::helper::argument_type<void(typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType)>::type >::type'
Foreach ($file in $FileNames)
{
  (Get-Content $file).replace($DirectiveOrign, $DirectiveAfter) | Set-Content $file
}

### C-style overloading disambiguation is not working properly with MSVC when it requires double template substitution.
#   Must patch line at 31 of bindings\python\algorithm\expose-geometry.cpp
$LineNumber = 31
$Contents = Get-Content $RootDir\pinocchio\bindings\python\algorithm\expose-geometry.cpp
Set-PSDebug -Trace 0
$Contents | Foreach {$n=1}{if ($LineNumber -eq $n) {
'(void (*)(GeometryModel &, const Eigen::MatrixBase<Vector3d> &))&setGeometryMeshScales<Vector3d>,'
} else {$_} ; $n++ } > $RootDir\pinocchio\bindings\python\algorithm\expose-geometry.cpp
Set-PSDebug -Trace 1

### Patch line 285 (289 v2.3.1 pinocchio) of \src\algorithm\model.hxx to fix dot placed after the closing double quote by mistake, not supported by MSVC.
$LineNumber = 285
$Contents = Get-Content $RootDir\pinocchio\src\algorithm\model.hxx
Set-PSDebug -Trace 0
$Contents | Foreach {$n=1}{if ($LineNumber -eq $n) {
'"The number of joints to lock is greater than the total of joints in the reduced_model.");'
} else {$_} ; $n++ } > $RootDir\pinocchio\src\algorithm\model.hxx
Set-PSDebug -Trace 1

### C-style overloading disambiguation is not working properly with MSVC.
#   Must patch lines at 35 of bindings\python\multibody\joint\joint-derived.hpp
$LineNumber = 35
$Contents = Get-Content $RootDir\pinocchio\bindings\python\multibody\joint\joint-derived.hpp
Set-PSDebug -Trace 0
$Contents | Foreach {$n=1}{if ($LineNumber -eq $n) {
'.def("setIndexes", bp::make_function(
                    (void (JointModelDerived::*)(JointIndex, int, int))&JointModelDerived::setIndexes,
                    bp::default_call_policies(),
                    boost::mpl::vector<void, JointModelDerived, JointIndex, int, int>()))'
} else {$_} ; $n++ } > $RootDir\pinocchio\bindings\python\multibody\joint\joint-derived.hpp
Set-PSDebug -Trace 1

### Build pinocchio, finally !
if (-not (Test-Path -PathType Container $RootDir\pinocchio\build)) {
  New-Item -ItemType "directory" -Force -Path "$RootDir\pinocchio\build"
}
Set-Location -Path $RootDir\pinocchio\build
cmake -G "Visual Studio 15" -T "v140" -DCMAKE_GENERATOR_PLATFORM=x64 -DCMAKE_CXX_STANDARD=11 -DCMAKE_INSTALL_PREFIX="$InstallDir" `
                                      -DBOOST_ROOT="$InstallDir" -DBoost_USE_STATIC_LIBS=OFF `
                                      -DBUILD_WITH_LUA_SUPPORT=OFF -DBUILD_WITH_COLLISION_SUPPORT=OFF -DBUILD_TESTING=OFF `
                                      -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_PYTHON_INTERFACE=OFF -DINSTALL_PYTHON_INTERFACE_ONLY=TRUE `
                                      -DCMAKE_CXX_FLAGS="/EHsc /bigobj -D_USE_MATH_DEFINES -DBOOST_ALL_NO_LIB -DBOOST_LIB_DIAGNOSTIC" $RootDir\pinocchio
cmake --build . --target install --config "$BuildType" --parallel 2
