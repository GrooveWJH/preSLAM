# PreSLAM

> The mathematics and algorithms, before starting (or during) SLAM learning.

This project explores fundamental mathematical concepts and algorithms relevant to Simultaneous Localization and Mapping (SLAM). A key feature is the side-by-side implementation of examples using both **traditional C++ (C++98/03 style)** and **modern C++ (C++11/14/17/20 features)**. This approach aims to showcase the evolution of C++ programming practices, highlighting the benefits of modern techniques in terms of expressiveness, safety, and maintainability.

## Problems Index

| Project                                               | Description                                                                                 |
| ----------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| [a0_solveMatrix](src/a0_solveMatrix)                     | Implementation of various linear equation solvers using Eigen (LU, Cholesky, QR, SVD, etc.) |
| [a1_pointDistance](src/a1_pointDistance)                 | Computing Euclidean distance between two points in N-dimensional space                      |
| [a2_poseTimeInterpolation](src/a2_poseTimeInterpolation) | Linear interpolation of poses in a time series                                              |
| [a3_a2-PLUS](src/a3_a2-PLUS)                             | Enhanced version of pose interpolation with template implementation                         |
| [a4_parallelization](src/a4_parallelization)             | Implementation of parallel for_each loop without external libraries                         |

## Prerequisites

* C++ Compiler (supporting C++20, e.g., Clang, GCC)
* CMake (version 3.10 or higher)
* Git

## Usage

1. **Clone the repository:**

   Since this project uses Git submodules (for the Eigen library), clone it recursively:

   ```bash
   git clone --recursive https://github.com/GrooveWJH/preSLAM.git
   cd preSLAM
   ```
   If you have already cloned the repository non-recursively, initialize the submodules:

   ```bash
   git submodule update --init --recursive
   ```
2. **Configure the project using CMake:**

   Create a build directory and run CMake from there:

   ```bash
   mkdir build
   cd build
   cmake .. 
   ```
3. **Compile the project:**

   Use `make` (or your chosen generator's build command) to compile the code. Using the `-j` flag allows parallel compilation, which can speed up the process.

   ```bash
   make -j 
   ```
4. **Run the examples:**

   The compiled executables are placed in the `output` directory within the project root, mirroring the structure of the `src` directory.

   Navigate to the output directory and run the desired example:

   ```bash
   # Example commands, run from the project root directory:
   ./output/a0_solveMatrix/a0_solveMatrix 
   ./output/a1_pointDistance/a1_pointDistance-modern
   ./output/a1_pointDistance/a1_pointDistance-traditional
   ./output/a2_poseTimeInterpolation/a2_poseTimeInterpolation-modern
   ./output/a2_poseTimeInterpolation/a2_poseTimeInterpolation-traditional
   ```
   You can also run the custom target `all_exps` during the build step to ensure all examples are built (though the default `make` target usually does this):

   ```bash
   cd build
   make all_exps
   ```
