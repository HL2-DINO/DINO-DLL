<p align=center>
  <img src="docs/img/dll_logo_light.png" width="95%/>
</div>

<html>
<body>
   <h3 align="center"><strong>HoloLens 2 &amp; <ins>D</ins>etection for <ins>I</ins>nfrared <ins>N</ins>avigation with <ins>O</ins>ST AR headsets</strong></p>
  
   <p align="center">
    <a href="#overview">Overview</a> •
    <a href="#3rd-party-dependencies">3rd Party Dependencies</a> •
    <a href="#getting-started">Getting Started</a> •
    <a href="#acknowledgements">Acknowledgements</a> •
    <a href="#appendix">Appendix</a>  
  </p>
</body>
</html>

## Overview
This is the C++ DLL used for the image-processing and tool-pose estimation used by the system reported in our publication: [Semi-Automatic Infrared Calibration for Augmented Reality Systems in Surgery](https://ieeexplore.ieee.org/document/9982215)[^1] by Hisham Iqbal & Ferdinando Rodriguez y Baena. 

This project is a Windows Runtime Component (C++/WinRT)[^2] DLL designed to enable the HoloLens 2 to detect the presence of tools equipped with IR-reflective markers. The resulting `.dll` and `.winmd` files can be copied and easily consumed on a C\# Unity app (see [`HL2-DINO-Unity`](https://github.com/HL2-DINO/DINO-Unity) for example).

<html>
<div align="center">
  <img src="docs/img/outputGifLabelled.gif" width="95%">
</div>
</html>

The classes in this project are designed to carry out the following tasks:

1. Grabbing sensor data from the HoloLens 2's AHAT depth sensor
1. Image processing to locate the presence of IR-reflective markers visible to the headset
1. Providing an interface for the C\# Unity application to receive sensor images, tool pose data etc.

For more details on each class, [check out the docs](https://hl2-dino.github.io/DINO-DLL/)!

## 3rd Party Dependencies

|Libraries/Headers  |Note                                                                                                                        |
|-------------------|----------------------------------------------------------------------------------------------------------------------------|
|`ResearchModeAPI.h`| From the [HoloLens2ForCV](https://github.com/microsoft/HoloLens2ForCV/tree/main/Samples/ResearchModeApi) project on GitHub |
|`OpenCV 4.9.0`     | Compiled for compatibility with UWP applications[^3]. Static libraries included within this project                        |
|`Eigen`            | Header-only matrix library with no extra compilation required                                                              |
|`Shiny-UWP`        | Used for gathering profiler data.[^4] Static libraries included.                                                           |

See the [NOTICE](./NOTICE) file in the repo for more details.

> **Note:** Shiny-UWP is optional and can be removed altogether if you are not interested in this info. 

## Getting Started
1. `git clone https://github.com/HL2-DINO/DINO-DLL.git`
1. Open `HL2DinoPlugin.sln`, set project configs to `Release` and `ARM64`
1. *[Optional] Make any changes to the source code as you wish*
1. From Visual Studio menus, `[Build] -> [Build Solution]`
1. Copy generated `.dll` and `.winmd` from the `ARM64/Release/HL2DinoPlugin/` folder into your Unity project inside the folder `Assets/Plugins/WSA` 

To see how to set up a sample Unity project, check out the [`DINO-Unity`](https://github.com/HL2-DINO/DINO-Unity) repositories.

You can use this project as is to build a `.dll` and `.winmd` in your Unity project. If you wish to make alterations to the source code to insert your own algorithms, then make any necessary adjustments, **and compile for `Release` and `ARM64` architecture**.

Two outputs of this process (`HL2DinoPlugin.dll` & `HL2DinoPlugin.winmd`) will be contained in `ARM64/Release/HL2DinoPlugin`. These need to be copied over to your Unity project structure inside the folder `Assets/Plugins/WSA`.

## Acknowledgements

* If this project is useful for your research or work, please considering citing the following publication:
  ```bibtex
  @inproceedings{Iqbal2022,
  author = {Hisham Iqbal and Ferdinando Rodriguez y Baena},
  journal = {2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2022)},
  title = {Semi-Automatic Infrared Calibration for Augmented Reality Systems in Surgery},
  year = {2022},
  }
  ```

* Thanks go out to:
  * Prof Ferdinando Rodriguez y Baena - For project supervision, endless advice, and for guidance with the point-matching algorithms used in the project.
  * [Andreas Keller](https://github.com/andreaskeller96) - For providing crucial help with proper compilation and linking of the OpenCV libraries used in this project. See Andreas's excellent [HoloLens2-IRTracking](https://github.com/andreaskeller96/HoloLens2-IRTracking) project for an alternative setup which achieves similar goals.
  * Arjun Muhunthan - For the many impromptu code-reviews I requested and for being a great source of C++ wisdom.
  * Dr Stefano Galvan - For his patience and time to provide best practices for software and algorithmic design as well as valuable insights into open-sourcing software.

* The design of the DLL in this project was based on some very useful open-source projects which are really worth exploring for your own research:
  1. [HoloLens2-ResearchMode-Unity](https://github.com/petergu684/HoloLens2-ResearchMode-Unity) originally made available by [petergu684](https://github.com/petergu684/). 
  2. [HoloLens2ForCV](https://github.com/microsoft/HoloLens2ForCV) originally made available by Microsoft.

>**Note**: [Gear Icon in repository logo created by Freepik - Flaticon](https://www.flaticon.com/free-icons/settings)

## Appendix 
### Additional Notes
There are some additional features which have not been implemented yet to expedite the process of open-sourcing. 

TO-DO's on the DLL side:
- [ ] Move away from detecting contours every single frame
    
    In this version, blob/contour detection runs every frame, which is the most time consuming part of the image processing loop. By using tracking algorithms to lower the frequency of detection, we could potentially get some performance gain. 

- [ ] Integrate filtering into the processing of the noisy sensor data. 

    Noise in the sensor data (infrared response / computed depth) results in noisy estimates for tool pose. This could be tackled with simple low-pass filters added to each TrackedTool struct, or an implementation of Kalman filters incorporating data from other streams of the HoloLens (visual light environmental cameras, IMU data).
  
- [ ] Adding support to process data in millimetres and metres. 

    The internal processing and exposed outputs of pose estimation is done entirely with metres. With some small scaffolding in the code, we can support doing this in millimetres. At present, you can pass in tool config data as millimetres, but it's converted into metres before it reaches the DLL. 

### License
This project is licensed under the [BSD License](LICENSE).

### Other Build Information
* The project was originally built to target Windows 10 SDK, version 2004 (10.0.19041.0).
* This project has been tested and built with VS 2019 & VS 2022, with Universal Windows Platform development tools installed.

***

[^1]: Iqbal H., Rodriguez y Baena, F. (2022) Semi‑Automatic Calibration for Augmented Reality Systems in Surgery.
2022 IEEE/RSJ International Conference on Intelligent Robots and Systems https://dx.doi.org/10.1109/IROS47612.2022.9982215

[^2]: The project was built using a Windows Runtime Component (C++/WinRT) project template on Visual Studio 2019. See [this tutorial](https://learn.microsoft.com/en-us/windows/uwp/winrt-components/create-a-windows-runtime-component-in-cppwinrt) for a reference on how to set up your own project. 

[^3]: If you want to re-compile OpenCV with different flags or for a different major version, it is highly recommended you follow the instructions found in this project [README](https://github.com/andreaskeller96/HoloLens2-IRTracking/blob/main/OpenCV/README.md) written by Andreas Keller. Some more reading on the topic can be done [here](https://medium.com/@rabbi.cse.sust.bd/how-to-build-opencv-for-universal-windows-platform-uwp-1a642ec09955) and [here](https://github.com/doughtmw/NuGet-Package-Creation)

[^4]: A [forked repository](https://github.com/hiqb217/shinyprofiler-uwp) of the original [Shiny Profiler](https://github.com/aidinabedi/shinyprofiler). 
