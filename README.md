# Mitsuba CLT
Mitsuba CLT extends the functionality of the [Mitsuba renderer](https://www.mitsuba-renderer.org/) (v0.5.0) by adding extensions for easier simulation of computational light transport imaging systems.

Currently implemented extensions include: perspective projector, orthographic projector, coded perspective camera and coded orthographic camera. 

## Gompiling the extended renderer

To compile Mitsuba CLT, you need to follow build instructions inside [Mitsuba documentation](https://www.mitsuba-renderer.org/releases/current/documentation.pdf), yet at the step of 
```
hg clone https://www.mitsuba-renderer.org/hg/mitsuba
```
you should instead clone this repository with following command:
```
git clone https://github.com/cmu-ci-lab/mitsuba_clt.git
```

## Basic usage

To use basic rendering functionality of Mitsuba, you can look into [Mitsuba documentation](https://www.mitsuba-renderer.org/releases/current/documentation.pdf). For the extended functionalities, you can consult below instructions to use projectors and coded cameras in the scene. You can also find some example scene files for using extended functionalities (modified from original Mitsuba sample scene files) in the examples/ directory.  

**Note that to use the new projector plugins with bidirectional path tracing, it is necessary to disable the lightImage option.** Below is an example of doing this,
```
<integrator type="bdpt">
    <boolean name="lightImage" value="false"/>
</integrator>
```

### Perspective Projector

A perspective projector constructor uses parameters similar to that of a perspective camera. Specifically, it includes all parameters of perspective camera except for the film. Also, it has extra parameters "filename"(string) and "scale"(float) to specify the image to be projected into the scene and the amount of scale the brightness of the projected image. For details pertaining to scene file creation of perspectiveprojector, please look into the documentation code at the very first few lines of the src/emitters/perspetctiveEmitterImpl.cpp file. 

### Orthographic Projector 

Similar to a perspective projector, a orthographic projector retains parameters of a orthographic camera except for film. Increased parameters are "filename"(string) and "irradiance"(float) which are used to specify projection image and a scale to amount of power per unit area received by a hypothetical surface normal to the specified direction. Further information of orthographic projector scene file creation can be found in src/emitters/orthographicEmitterImpl.cpp

### Coded Perspective Camera and Coded Orthographic Camera

An additional parameter "filename(string)" is added to both coded cameras compared to their versions without masks. The filename is used as a path to the image file that contains the mask encoding the camera. If the image has resolution inconsistent with the camera film, the mask will be scaled to fit the size of the film. There is no default value for filename, so this is a required parameter for coded cameras. You can view more detailed documentations of constructing coded cameras in src/sensors/codedOrthographic.cpp and src/sensors/codedPerspective.cpp

### "filename" parameter is required for all extended features, so please make sure to always include it scene files for using projectors or coded cameras. 

### Light Transport Probing
Three kinds of probing "identity", "row" and "column" are available and they **work only when using bdpt integrator**.

**Identity Probing**: camera's pixel whose image plane index is (i,j) would sample light from only one pixel(i+a,j+b) that has certain displacement(a,b) from the camera pixel position on the projector.

To use identity probing, you must contain one perspective projector and one perspective camera in the scene. Also, you need to set in integrator the type of probing to identity as below:
```
<integrator type="bdpt">
    <boolean name="lightImage" value="false"/>
    <string name="type" value="identity"/>
</integrator>
```
The lightImage also needs to be turned off, since we make use of perspective projector.
The displacement of pixels sampled from a projector is by default 0 in both x and y dimension of the projector's image plane. However, you can set the displacement of sampled pixels on perspective projector as below:
```
<emitter type="perspectiveprojector">
    <float name="farClip" value="2800"/>
    <float name="nearClip" value="10"/>
    ...
    <integer name="rowDisplacement" value="VALUEYOUINTENDED"/>
    <integer name="colDisplacement" value="VALUEYOUINTENDED"/>
</emitter>
```

**Row Probing**: camera's pixel whose image plane index is (i,j) would sample light from only pixels on row i+a where a is the displacement from the camera pixel's row on the projector.

Similar to the usage of identity probing, when using row probing, you must contain one perspective projector and one perspective camera in the scene. Also, you need to set in integrator the type of probing to row as below:
```
<integrator type="bdpt">
    <boolean name="lightImage" value="false"/>
    <string name="type" value="row"/>
</integrator>
```
The lightImage also needs to be turned off, since we make use of perspective projector.
The displacement of row sampled from a projector is by default 0 in the projector's image plane. However, you can set the displacement of rows on perspective projector as below:
```
<emitter type="perspectiveprojector">
    <float name="farClip" value="2800"/>
    <float name="nearClip" value="10"/>
    ...
    <integer name="rowDisplacement" value="VALUEYOUINTENDED"/>
</emitter>
```

**Column Probing**: camera's pixel whose image plane index is (i,j) would sample light from only pixels on row j+b where b is the displacement from the camera pixel's column on the projector's image plane.

Similar to the usage of identity and row probing, when using column probing, you must contain one perspective projector and one perspective camera in the scene. Also, you need to set in integrator the type of probing to column as below:
```
<integrator type="bdpt">
    <boolean name="lightImage" value="false"/>
    <string name="type" value="column"/>
</integrator>
```
The lightImage also needs to be turned off, since we make use of perspective projector.
The displacement of column sampled from a projector is by default 0 in the projector's image plane. However, you can set the displacement of columns on perspective projector as below:
```
<emitter type="perspectiveprojector">
    <float name="farClip" value="2800"/>
    <float name="nearClip" value="10"/>
    ...
    <integer name="colDisplacement" value="VALUEYOUINTENDED"/>
</emitter>
```

### Disparity Camera ###
Disparity Camera renders an image that shows the difference each pixel's index on the image plane and its sampled projector's pixel's index. Speficially, the r channel of the image shows the displacement of projector pixel's column index from the camera pixel's column index and the g channel of the image shows the displacement of projector pixel's row index from the camera pixel's row index.

To use disparity camera, you must contain one perspective projector and one perspective camera in the scene. Also, you need to set in integrator's type parameter to disparity as below:
```
<integrator type="bdpt">
    <string name="type" value="disparity"/>
</integrator>
```
## Authors

The project is derived from the [Mitsuba renderer](https://www.mitsuba-renderer.org/), written by Wenzel Jakob et al. See the Mitsuba website for a full list of contributors.

The authors of the Mitsuba CLT extensions are:
- [**Jiatian (Caroline) Sun**](https://jiatiansun.github.io/) (primary author)
- [Ioannis Gkioulekas](http://www.cs.cmu.edu/~igkioule/)
