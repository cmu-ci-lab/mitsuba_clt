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

**Note that to use new projector plugins with bidirectional path tracing, it is necessary to disable the lightImage option.** Below is an example of doing this,
```
<integrator type="bdpt">
    <boolean name="lightImage" value="false"/>
</integrator>
```

### Perspective Projector

| Parameter | Type | Description |
|:---------|:-----------|:----------|
| toWorld | transform or animation | Specifies an optional camera-to-world transformation. <br>(Default: none(i.e. camera space = world space)) |
|  focalLength  |   string|   Denotes the camera’s focal length specified using 35mm film equivalent units. See the description below for further details. (Default:50mm) | 
|  fov | float | An alternative to focalLength: denotes the camera’s field of view in degrees—must be between 0 and 180,<br> excluding the extremes.|
| fovAxis |  string   | When the parameter fov is given (and only then), this parameter further specifies the image axis, to which it applies.<br> (i) **x**: fov maps to the x-axis in screen space. <br> (ii) **y**: fov maps to the y-axis in screen space. <br> (iii) **diagonal**: fov maps to the screen diagonal.<br> (iv) **smaller**: fov maps to the smaller dimension (e.g. x when width<height) <br> (v) **larger**: fov maps to the larger dimension (e.g. y when width<height) <br> The default is **x**.  |
| nearClip,<br> farClip | float | Distance to the near/far clip planes.<br>(Default: nearClip=1e-2 (i.e. 0.01) and farClip=1e4 (i.e. 10000) |
| filename | string | Path to the radiance-valued input image to be projected |
| scale | float | A scale factor that is applied to the radiance values stored <br>in theinputimage. (Default:1) |

This plugin impelements a perspective projector model, assuming the same idealized pinhole perspective camera model as Mitsuba's `perspective` sensor plugin. Thus, as described in the documentation of Mitsuba, By default, the projector’s field of view is specied using a 35mm film equivalent focal length, which is first converted into a diagonal feld of view and subsequently applied to the projector. is assumes that the projecting image’s aspect ratio matches that of 35mm film (1.5:1), though the parameter still behaves intuitively when this is not the case. Alternatively, it is also possible to specify a field of view in degrees along a given axis (see the fov and fovAxis parameters).

Also, it has extra parameters `filename` and `scale` to specify correspondingly the image to be projected onto the scene and how much should the brightness of the projected image be scaled. 

Below is sample configuration needed for instantiating a perspective projector in a scene file. 
```
    <emitter type="perspectiveprojector">
        <!-- .... other perspective projector parameters (that has counterpart in persepective camera) .... -->
        <string name="filename" value="PathToProjectionImage"/>
        <float name="scale" value="1000"/>
    </emitter>
```    
For example application of this plugin, you can look into the file `examples/cornellbox/cbox_perspective_proj.xml`.

### Orthographic Projector 

| Parameter | Type | Description |
|:---------|:-----------|:----------|
| toWorld | transform or animation | Specifies an optional camera-to-world transformation. <br>(Default: none(i.e. camera space = world space)) |
| nearClip,<br> farClip | float | Distance to the near/far clip planes.<br>(Default: nearClip=1e-2 (i.e. 0.01) and farClip=1e4 (i.e. 10000) |
| filename | string | Path to the radiance-valued input image to be projected |
| scale | float | A scale factor that is applied to the radiance values stored <br>in theinputimage. (Default:1) |

This plugin implements a orthographic projector. It can be thought of as a planar emitter that sends out light only in its normal direction. Similar to the perspective projector plugin, orthographic projector retains most of parameters of Mitsbua's orthographic camera plugin. Extra parameters "filename"(string) and "scale"(float) correspondingly specify projection image and a scale factor that is applied to the radiance values stored in the input image. 

Below is sample configuration needed for instantiating a orthographic projector in a scene file. 
```
    <emitter type="orthographicprojector">
        <!-- .... other orthographic projector parameters (that has counterpart in orthographic camera) .... -->
        <string name="filename" value="PathToProjectionImage"/>
        <float name="scale" value="1000"/>
    </emitter>
```

For example application of this plugin, you can look into the file `examples/cornellbox/cbox_orthographic_proj.xml`.

### Coded Perspective Camera 

| Parameter | Type | Description |
|:---------|:-----------|:----------|
| toWorld | transform or animation | Specifies an optional camera-to-world transformation. <br>(Default: none(i.e. camera space = world space)) |
|  focalLength  |   string|   Denotes the camera’s focal length specified using 35mm film equivalent units. See the description below for further details. (Default:50mm) | 
|  fov | float | An alternative to focalLength: denotes the camera’s field of view in degrees—must be between 0 and 180,<br> excluding the extremes.|
| fovAxis |  string   | When the parameter fov is given (and only then), this parameter further specifies the image axis, to which it applies.<br> (i) **x**: fov maps to the x-axis in screen space. <br> (ii) **y**: fov maps to the y-axis in screen space. <br> (iii) **diagonal**: fov maps to the screen diagonal.<br> (iv) **smaller**: fov maps to the smaller dimension (e.g. x when width<height) <br> (v) **larger**: fov maps to the larger dimension (e.g. y when width<height) <br> The default is **x**.  |
| nearClip,<br> farClip | float | Distance to the near/far clip planes.<br>(Default: nearClip=1e-2 (i.e. 0.01) and farClip=1e4 (i.e. 10000) |
| filename | string | Path to the radiance-valued input image to be projected (Required for initialization)|


This plugin implements a coded perspective pinhole camera, which simply applies a mask to a pinhole of a perspective camera.<br>. Thus, coded perspective camera reserves most of parameters of Mitsbua's perspective camera plugin. An additional parameter "filename(string)" is added to the coded camera compared to its version without masks. The filename is used as a path to the image file that contains the mask that filters the image rendered by the camera. If the image has resolution inconsistent with the camera film, the mask will be scaled to fit the size of the film. There is no default value for filename, so this is a required parameter for coded cameras. 

Below is sample configuration needed for instantiating a coded perspective camera in a scene file. 
```
	<sensor type="codedPerspective">
		 <!-- .... other coded perspective camera parameters (that has counterpart in persepective camera) .... -->
		<string name="filename" value="PathToProjectionImage"/>
	</sensor>
```
For example application of this plugin, you can look into the file `examples/cornellbox/cbox_coded_persp_proj.xml`.

### Coded Orthographic Camera

| Parameter | Type | Description |
|:---------|:-----------|:----------|
| toWorld | transform or animation | Specifies an optional camera-to-world transformation. <br>(Default: none(i.e. camera space = world space)) |
| nearClip,<br> farClip | float | Distance to the near/far clip planes.<br>(Default: nearClip=1e-2 (i.e. 0.01) and farClip=1e4 (i.e. 10000) |
| filename | string | Path to the radiance-valued input image to be projected (Required for initialization) |

This plugin implements a coded orthographic camera, which simply applies a mask to a pinhole of a orthographic camera.<br>. Thus, coded orthographic camera reserves most of parameters of Mitsbua's orthographic camera plugin. An additional parameter "filename(string)" is added to the coded camera compared to its version without masks. The filename is used as a path to the image file that contains the mask that filters the image rendered by the camera. If the image has resolution inconsistent with the camera film, the mask will be scaled to fit the size of the film. There is no default value for filename, so this is a required parameter for coded cameras. 

Below is sample configuration needed for instantiating a coded perspective camera in a scene file. 
```
	<sensor type="codedOrthographic">
		 <!-- .... other coded perspective camera parameters (that has counterpart in persepective camera) .... -->
		<string name="filename" value="PathToProjectionImage"/>
	</sensor>
```
For example application of this plugin, you can look into the file `examples/cornellbox/cbox_coded_ortho_proj.xml`.

### "filename" parameter is required for all plugins above, so please make sure to always include it scene files for using projectors or coded cameras. 

### Light Transport Probing
Three kinds of probing "identity", "row" and "column" are available and they **work only when using bdpt integrator and perspective projector**.

**Identity Probing**: In this plugin, we limit each perspective camera's pixel denoted (i,j) to evaluate light paths originated from only one pixel(i+a,j+b) on the projector that has displacement(a,b) from the camera's pixel position(i,j).

To use identity probing, you must contain exactly one perspective projector and one perspective camera in the scene. Also, you need to set the integrator and the probing type to identity as below:
```
<integrator type="bdpt">
    <boolean name="lightImage" value="false"/>
</integrator>

<probe type="identity">
    <disparity x="X_DISPLACEMENT" y="Y_DISPLACEMENT"/>
</probe>
```
As shown in the code above, the lightImage also needs to be turned off for the bdpt integrator. <br> You can set the disparity between sampled pixels of a camera and a projector by specifying "x" and "y" attributes of disparity. If you ignore declaring disparity like `<probe type="identity"/>` or ignoring declaration of disparity in one dimension like <br>
```
<probe type="identity">
    <disparity x="X_DISPLACEMENT"/>
</probe>
```
, ignored disparity dimensions are by default 0. 

**Row Probing**: In this plugin, we limit each perspective camera's pixel denoted (i,j) to evaluate light paths originated from only pixels on row i+a of a perspective projector where a is the displacement from the camera pixel's row.

Similar to the usage of identity probing, when using row probing, you must contain one perspective projector and one perspective camera in the scene. Also, you need to set the integrator to be `bdpt` and turn off `lightImage` as below:
```
<integrator type="bdpt">
    <boolean name="lightImage" value="false"/>
</integrator>
```
Besides, you need to set the probe type to be `row` and `disparity` as follows:
```
<probe type="row">
    <disparity y="Y_DISPLACEMENT"/>
</probe>
```
The displacement in y dimension between projector and the camera is by default 0, if you ignore explicitly the y value of disparity like `<probe type="row"/>`.

**Column Probing**: In this plugin, we limit each perspective camera's pixel denoted (i,j) to evaluate only light paths originated from pixels on column j+b of a perpsetctive projector where b is the displacement from the camera pixel's column.

Similar to the usage of identity and row probing, when using column probing, you must contain one perspective projector and one perspective camera in the scene. Also, you need to set the integrator to be `bdpt` and turn off `lightImage` as below:
```
<integrator type="bdpt">
    <boolean name="lightImage" value="false"/>
</integrator>
```
Besides, you need to set the probe type to be `column` and `disparity` as follows:
```
<probe type="column">
    <disparity x="X_DISPLACEMENT"/>
</probe>
```
The displacement in x dimension between projector and the camera is by default 0, when you ignore explicitly the y value of disparity like `<probe type="column"/>`.


**epipolar Probing**: This plugin evaluates only light paths whose perspective camera and perspective projector's samples are on a same plane with origins of the camera and the projector.

When using epipolar probing, you must contain exactly one perspective projector and exactly one perspective camera in the scene. Also, you need to set the integrator to be `bdpt` and turn off both `lightImage` and `sampleDirect` as below:
```
<integrator type="bdpt">
    <boolean name="lightImage" value="false"/>
    <boolean name="sampleDirect" value="false"/>
</integrator>
```
Besides, you need to set the probe type to be `epipolar`as follows:
```
<probe type="epipolar"/>
```

## Authors

The project is derived from the [Mitsuba renderer](https://www.mitsuba-renderer.org/), written by Wenzel Jakob et al. See the Mitsuba website for a full list of contributors.

The authors of the Mitsuba CLT extensions are:
- [**Jiatian (Caroline) Sun**](https://jiatiansun.github.io/) (primary author)
- [Ioannis Gkioulekas](http://www.cs.cmu.edu/~igkioule/)
