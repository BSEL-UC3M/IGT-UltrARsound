# IGT-UltrARsound

## Overview
This Unity project is based on the paper [UltrARsound: in situ visualization of live ultrasound images using HoloLens 2](https://link.springer.com/article/10.1007/s11548-022-02695-z) and shows 

- how to track a rigid body based on retroreflective spheres using the depth camera and the AHAT (short-throw) mode with HoloLens 2.
- how to stream ultrasound images from a PLUS compatible ultrasound station and visualize them

**Note:** The repository is still work in progress

## Prerequisites and setting the project up
1. [Enable research mode and allow access to sensor stream](https://docs.microsoft.com/en-us/windows/mixed-reality/develop/advanced-concepts/research-mode#enabling-research-mode-hololens-first-gen-and-hololens-2)
2. [OpenCV for Unity from Asset Store](https://assetstore.unity.com/packages/tools/integration/opencv-for-unity-21088)
**Note:** There might be other free OpenCV libraries for Unity and HoloLens but they might use different names for methods etc.
3. Open the Unity project and import OpenCV for Unity (there should be no errors after importing)
4. Use the Mixed Reality Feature Tool and import MRTK Foundation Feature
5. Open the scene "UltrARsound"
6. Import both prefabs in the Prefabs folder
7. Under "Buttons" --> "ButtonCollection"
	- "Button4" in Events and OnClick add the USimageStreamer and select the function OpenIGTLinkConnect.ConnectUS method
	- "Button5" in Events and OnClick add the RigidBodyTrackingController and select the function RigidBodyTracker.UseKalmanFilter method
	- "Button6" in Events and OnClick add the RigidBodyTrackingController and select the function RigidBodyTracker.StartTrackingCoroutine method

 


## General information for usage

- The script *RigidBodyTracker.cs* performs the actual tracking of the spheres and calculates the pose of the rigid body. Within the method *defineRigidBody()*, you can define the positions of your spheres with respect to your reference frame.
- The script *OpenIGTLinkConnect.cs* performs the streaming and visualization of ultrasound images. It is attached to the *USimage* game object and in the inspector it is possible to adjust ip address and port number.
- Once the application is deployed to HoloLens 2 and the app is started, you should the live stream of the active brightness image. Only if you can see the live stream, you can be sure that the tracking will work. 
- The user interface with three buttons enables to start streaming the ultrasound images, start the tracking and turn on and off the Kalman filter.
 
- For further questions, please contact vonhaxthausen@rob.uni-luebeck.de

## Citation
If you find our work useful, please consider citing our paper
```
	@article{VonHaxthausen2022,
		author = {von Haxthausen, Felix and Moreta-Martinez, Rafael and {Pose D{\'{i}}ez de la Lastra}, Alicia and Pascau, Javier and Ernst, Floris},
		doi = {10.1007/s11548-022-02695-z},
		issn = {1861-6429},
		journal = {International Journal of Computer Assisted Radiology and Surgery},
		title = {{UltrARsound: in situ visualization of live ultrasound images using HoloLens 2}},
		url = {https://doi.org/10.1007/s11548-022-02695-z},
		year = {2022}
	}

```

### Acknowledgement
This repository borrows code from [GU]. Please acknowledge his work if you find this 
repo useful! 

[GU]: https://github.com/petergu684/HoloLens2-ResearchMode-Unity
