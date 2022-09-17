# ICP_algorithm
Iterative Closest Point algorithm for fine alignment of point clouds

# Iterative Closest Point Algorithm
<p align="center">
 <strong>Comparison of a few Iterative closest point algorithms and their implementation.</strong> 
  <br>
    <img src="https://img.shields.io/badge/C++-orange.svg"/>
    <img src="https://img.shields.io/badge/CMake-brightgreen.svg"/>
    <img src="https://img.shields.io/badge/PCL-ff69b4.svg"/>
    <img src="https://img.shields.io/badge/ICP-blue.svg"/>
  <br>
</p>

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
    - [Manually](#manually)

- [Usage](#usage)
- [Note](#note)
- [References](#references)
- [License](#license)


## Introduction
This file contains the implementaion of ICP algorithm which finds the fine alignment transformation matrix for two given input point cloud files along with a heuristic, which is a coarse transformation matrix.

## Prerequisites

Make sure to have:
- `CMake` ([installation guide](https://cmake.org/install/))

## Installation
### Manually
- Create a `build` directory and move into it
``` sh
sh$ mkdir build
sh$ cd build
```
- Generate the Makefile with `CMake`
``` sh
sh$ build/cmake ..

```
- Generate the binary file `runfile`
``` sh
sh$ build/make
```

- Run the compiled file
``` sh
sh$ build/./runfile
```


## Usage
[Video](https://youtu.be/idQaYfb7TCc) showing the visualisation of ICP algorithm in action
## Note
*I'm adding some of the my personal notes and findings about this topic*
- pcl has GeneralisedICP, NonLinearICP and ICP. 
  - ICP algorithm transformation is estimated based on Singular Value Decomposition (SVD)
  - ICP_NL is an ICP variant that uses Levenberg-Marquardt optimization backend. The resultant transformation is optimized as a quaternion.
  - Generalised_ICP is based on this [paper](http://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf) In this paper authors have combined     the the Iterative Closest Point (’ICP’) and ‘point-to-plane ICP‘ algorithms into a single probabilistic framework.
- 


## References
- ICP algorithm of PCL([Documentation](https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html))
- formating reference of this readme.md referred from [yassram](https://github.com/yassram/iterative-closest-point#readme)  
- 

## License
```
MIT License

Copyright (c) 2020 yassir RAMDANI

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
