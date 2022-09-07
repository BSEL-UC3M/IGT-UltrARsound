using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using System.IO;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.UnityUtils;
using System.Threading.Tasks;
using Ultrasound.Tracking;
using System.Globalization;
using System.Threading;



#if ENABLE_WINMD_SUPPORT
using HL2UnityPlugin;
#endif

public class RigidBodyTracker : MonoBehaviour
{

#if ENABLE_WINMD_SUPPORT
    HL2ResearchMode researchMode;
#endif

    private byte[] depthData = null;
    private ushort[] depthValues;

    // preview of short throw active brightness image - good to check if stream is working
    public GameObject shortAbImagePreviewPlane = null;
    private Material shortAbImageMediaMaterial = null;
    private Texture2D shortAbImageMediaTexture = null;
    private byte[] shortAbImageFrameData = null;

    // A static image of a retroreflective spheres - used for debugging within the unity editor
    public GameObject originalImagePreviewPlane = null;
    private Material originalImageMediaMaterial = null;
    private Texture2D originalImageMediaTexture = null;
    private byte[] originalImageFrameData = null;


    byte[] bytesLUT;
    private float[] LUT;
    private Mat matLUT;
    private Mat matLUTimage;
    private Mat matLUTimageRot;
    private Mat depthImage;
    private List<float> blobLocations;



    Matrix4x4 depthCameraExtrinsics;
    float[] poseRigidNode;
    float[] poseRigidNodeTemp;

    MatOfKeyPoint blobs;
    Mat im_with_keypoints;

    int[] imSize = new int[2] { 512, 512 };

    TransformHorn3D horn;
    private float[] distancesSpheres;
    private float maxDistanceTolerance = 0.01f;

    private List<Vector3> posSpheresInCamSpaces;
    public bool useCorrectionVector = true;
    public bool useKalmanFilter = false;
    public GameObject rigidBody;

    private int[] blobDiameterSize;
    bool isBlobDetectionFinished = false;
    bool isTrackingFinished = true;
    bool iscalculatedPosInCamSpace = false;
    int numberOfSpheresForRigidBody;

    ushort[] frameDepthMap = null;
    float frequency = 10;

    private OpenCVForUnity.Features2dModule.SimpleBlobDetector_Params paramtersBlobDetetcor;
    private OpenCVForUnity.Features2dModule.SimpleBlobDetector blobDetector;


    KalmanFilterSphere[] KFspheres;


    void Start()
    {

        KFspheres = new KalmanFilterSphere[4];
        for (int i = 0; i < KFspheres.Length; i++)
        {
            KFspheres[i] = new KalmanFilterSphere();
        }

        for (int i = 0; i < KFspheres.Length; i++)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.tag = "sphere";
            sphere.transform.localScale = new Vector3((float)0.007, (float)0.007, (float)0.007);
        }

#if UNITY_EDITOR
        // load prerecorded data from a depth image in the unity editor --> for debugging
        var dephtValuesTextAsset = Resources.Load("depthValuesString") as TextAsset;
        string depthValuesString = dephtValuesTextAsset.text;
        var stringseperatedt = (depthValuesString.Split(','));
        depthValues = new ushort[stringseperatedt.Length];
        for (int i = 0; i < stringseperatedt.Length; i++)
        {
            depthValues[i] = ushort.Parse(stringseperatedt[i]);
        }
#endif

        paramtersBlobDetetcor = new OpenCVForUnity.Features2dModule.SimpleBlobDetector_Params();
        paramtersBlobDetetcor.set_filterByCircularity(true);
        paramtersBlobDetetcor.set_filterByConvexity(false);
        paramtersBlobDetetcor.set_filterByInertia(true);
        paramtersBlobDetetcor.set_filterByArea(true);

        paramtersBlobDetetcor.set_minArea(8);
        paramtersBlobDetetcor.set_maxArea(300);
        paramtersBlobDetetcor.set_minCircularity((float)0.85);
        paramtersBlobDetetcor.set_maxCircularity((float)2);
        paramtersBlobDetetcor.set_minThreshold((float)0);
        paramtersBlobDetetcor.set_maxThreshold((float)180);
        paramtersBlobDetetcor.set_minInertiaRatio((float)0.4);
        blobDetector = OpenCVForUnity.Features2dModule.SimpleBlobDetector.create(paramtersBlobDetetcor);

        // extrinsic parameters from left front camera to the depth camera
        Vector4 column1Extrinsics = new Vector4(0.0189669f, -0.999739f, -0.012767f, -0.0592664f);
        Vector4 column2Extrinsics = new Vector4(0.95834f, 0.0218184f, -0.284794f, -0.0150002f);
        Vector4 column3Extrinsics = new Vector4(0.284998f, -0.00683347f, 0.958504f, -0.0181255f);
        Vector4 column4Extrinsics = new Vector4(0, 0, 0, 1);
        depthCameraExtrinsics = new Matrix4x4(column1Extrinsics, column2Extrinsics, column3Extrinsics, column4Extrinsics);
        depthCameraExtrinsics = depthCameraExtrinsics.transpose;



        // Load LookUp table 
        var loadedLUT = Resources.Load("AHaT_lut") as TextAsset;
        bytesLUT = loadedLUT.bytes;
        LUT = new float[512 * 512 * 3];
        Buffer.BlockCopy(bytesLUT, 0, LUT, 0, 512 * 512 * 3 * 4); // LUT is 512*512*4*3 because float --> 4 bytes
        //Debug.Log("LUT is long:" + LUT.Length); 
        matLUT = new Mat(512 * 512, 3, CvType.CV_32F);
        matLUT.put(0, 0, LUT);
        matLUTimage = matLUT.reshape(3, imSize);
        matLUTimageRot = new Mat();
        Core.rotate(matLUTimage, matLUTimageRot, 0);



        shortAbImageMediaMaterial = shortAbImagePreviewPlane.GetComponent<MeshRenderer>().material;
        shortAbImageMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
        shortAbImageMediaMaterial.mainTexture = shortAbImageMediaTexture;


        originalImageMediaMaterial = originalImagePreviewPlane.GetComponent<MeshRenderer>().material;
        originalImageMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
        originalImageMediaTexture = (Texture2D)originalImagePreviewPlane.GetComponent<Renderer>().material.mainTexture;
        originalImageMediaMaterial.mainTexture = originalImageMediaTexture;




#if ENABLE_WINMD_SUPPORT
        researchMode = new HL2ResearchMode();
        researchMode.InitializeDepthSensor();
        //researchMode.InitializeSpatialCamerasFront();

        researchMode.SetPointCloudDepthOffset(0);

        // Depth sensor should be initialized in only one mode
        researchMode.StartDepthSensorLoop();
        //researchMode.StartLongDepthSensorLoop(); 
        
        //researchMode.StartSpatialCamerasFrontLoop();
#endif

        defineRigidBody();

    }

    void defineRigidBody()
    {
        List<float[]> positionsInReferenceFrame = new List<float[]>();
        List<Vector3> positionsInReferenceFrameAsVector3 = new List<Vector3>();

#if UNITY_EDITOR
        float[] points = new float[3];
        points[0] = -0.05055f;
        points[1] = -0.03029f;
        points[2] = 0.01620f;
        positionsInReferenceFrame.Add(points);
        positionsInReferenceFrameAsVector3.Add(new Vector3(points[0], points[1], points[2]));

        points = new float[3];
        points[0] = -0.03240f;
        points[1] = 0.01094f;
        points[2] = 0.01335f;
        positionsInReferenceFrame.Add(points);
        positionsInReferenceFrameAsVector3.Add(new Vector3(points[0], points[1], points[2]));


        points = new float[3];
        points[0] = 0.03895f;
        points[1] = -0.01912f;
        points[2] = -0.01665f;
        positionsInReferenceFrame.Add(points);
        positionsInReferenceFrameAsVector3.Add(new Vector3(points[0], points[1], points[2]));


        points = new float[3];
        points[0] = 0.04399f;
        points[1] = 0.03846f;
        points[2] = -0.01289f;
        positionsInReferenceFrame.Add(points);
        positionsInReferenceFrameAsVector3.Add(new Vector3(points[0], points[1], points[2]));
#else
// Polaris Stylus
       float[] points = new float[3];
        points[0] = 0f;
        points[1] = 0f;
        points[2] = 0f;
        positionsInReferenceFrame.Add(points);
        positionsInReferenceFrameAsVector3.Add(new Vector3(points[0], points[1], points[2]));

        points = new float[3];
        points[0] = 0f;
        points[1] = 0f;
        points[2] = 0.05f;
        positionsInReferenceFrame.Add(points);
        positionsInReferenceFrameAsVector3.Add(new Vector3(points[0], points[1], points[2]));


        points = new float[3];
        points[0] = 0f;
        points[1] = 0.025f;
        points[2] = 0.10f;
        positionsInReferenceFrame.Add(points);
        positionsInReferenceFrameAsVector3.Add(new Vector3(points[0], points[1], points[2]));


        points = new float[3];
        points[0] = 0f;
        points[1] = -0.025f;
        points[2] = 0.135f;
        positionsInReferenceFrame.Add(points);
        positionsInReferenceFrameAsVector3.Add(new Vector3(points[0], points[1], points[2]));
#endif

        numberOfSpheresForRigidBody = positionsInReferenceFrame.Count;
        distancesSpheres = new float[positionsInReferenceFrame.Count];
        calculateDistancesofSpheres(positionsInReferenceFrameAsVector3);
        horn = new TransformHorn3D(positionsInReferenceFrame);
    }

    void calculateDistancesofSpheres(List<Vector3> positionsSpheres)
    {
        distancesSpheres[0] = 0f;
        distancesSpheres[1] = Vector3.Distance(positionsSpheres[0], positionsSpheres[1]);
        distancesSpheres[2] = Vector3.Distance(positionsSpheres[0], positionsSpheres[2]);
        distancesSpheres[3] = Vector3.Distance(positionsSpheres[0], positionsSpheres[3]);
    }


    bool startRealtimePreview = true;
    void LateUpdate()
    {
#if ENABLE_WINMD_SUPPORT
        // update depth map texture
        if (startRealtimePreview && researchMode.DepthMapTextureUpdated())
        {
            frameDepthMap = researchMode.GetDepthMapBuffer();
        }
        // update short-throw AbImage texture --> active brightness image
        if (startRealtimePreview && researchMode.ShortAbImageTextureUpdated())
        {
            byte[] frameTexture = researchMode.GetShortAbImageTextureBuffer();
            if (frameTexture.Length > 0)
            {
                if (shortAbImageFrameData == null)
                {
                    shortAbImageFrameData = frameTexture;
                }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, shortAbImageFrameData, 0, shortAbImageFrameData.Length);
                }

                shortAbImageMediaTexture.LoadRawTextureData(shortAbImageFrameData);
                shortAbImageMediaTexture.Apply();
                poseRigidNodeTemp = researchMode.GetRigidNodePose();

            }
        }
#endif
    }




    private void FixedUpdate()
    {

        //        if (isBlobDetectionFinished)
        //        {
        //#if UNITY_EDITOR // in Unity with prerecorded image and on HoloLens with real data
        //            calculatePosSphere(depthValues, blobLocations);
        //#else
        //            calculatePosSphere(frameDepthMap, blobLocations);
        //#endif
        //            isBlobDetectionFinished = false;
        //        }

        //        if (iscalculatedPosInCamSpace)
        //        {
        //            createSphereBasedOnLocAndCalcRigidBody(blobLocations);
        //            iscalculatedPosInCamSpace = false;
        //            isTrackingFinished = true;
        //        }
    }

    void calculatePosSphere(ushort[] depthMapArray, List<float> listBlobsInImage)
    {
        posSpheresInCamSpaces = new List<Vector3>();
        var depthMapArrayShort = Array.ConvertAll(depthMapArray, item => (short)item);
        Mat matDepthImage = new Mat(512, 512, CvType.CV_16U);
        matDepthImage.put(0, 0, depthMapArrayShort);
        Core.rotate(matDepthImage, matDepthImage, 0);

        for (int i = 0; i < listBlobsInImage.Count; i = i + 2)
        {
            int x_pixel = Convert.ToInt32(Math.Round(listBlobsInImage[i]));
            int y_pixel = Convert.ToInt32(Math.Round(listBlobsInImage[i + 1]));
            var depthMapScalar2 = matDepthImage.get(x_pixel, y_pixel);
            var uvw = matLUTimageRot.get(x_pixel, y_pixel);
            var x = uvw[0] * depthMapScalar2[0];
            var y = uvw[1] * depthMapScalar2[0];
            var z = uvw[2] * depthMapScalar2[0];

            Vector3 cam2point = new Vector3((float)x / 1000, (float)y / 1000, (float)z / 1000);
            posSpheresInCamSpaces.Add(cam2point);
        }
        iscalculatedPosInCamSpace = true;
        return;
    }


    List<float> getBlobsInImage(Mat imgMat)
    {
        blobs = new MatOfKeyPoint();
        blobDetector.detect(imgMat, blobs);
        var blobArray = blobs.toArray();
        List<float> listBlobsLoc = new List<float>();
        double[] tempLocsArray = new double[6];
        blobDiameterSize = new int[blobArray.Length];

        for (int i = 0; i < blobArray.Length; i++)
        {
            blobDiameterSize[i] = (int)(Math.Round(blobArray[i].size));
            tempLocsArray = blobs.get(i, 0);
            listBlobsLoc.Add((float)tempLocsArray[0]);
            listBlobsLoc.Add((float)tempLocsArray[1]);
        }
        return listBlobsLoc;
    }



    IEnumerator StartTrackingWithCoroutine()
    {
        while (true)
        {
            yield return null;
            ////yield return new WaitForSeconds(1 / frequency); // Frequency of tracking
            if (isTrackingFinished)
            {
                isTrackingFinished = false;
#if UNITY_EDITOR
                Mat imgMat = new Mat(originalImageMediaTexture.height, originalImageMediaTexture.width, CvType.CV_8UC1);
                Utils.texture2DToMat(originalImageMediaTexture, imgMat);
                Core.rotate(imgMat, imgMat, 1);
                Core.flip(imgMat, imgMat, 1);
#else
                Mat imgMat = new Mat(shortAbImageMediaTexture.height, shortAbImageMediaTexture.width, CvType.CV_8UC1);
                Utils.texture2DToMat(shortAbImageMediaTexture, imgMat);
                Core.rotate(imgMat, imgMat, 1);
                Core.flip(imgMat, imgMat, 1);
                poseRigidNode = poseRigidNodeTemp;
                OpenCVForUnity.CoreModule.Core.bitwise_not(imgMat, imgMat);
#endif

                Core.flip(imgMat, imgMat, 0);

                Task<List<float>> taskBlobDetection = Task.Factory.StartNew(() =>
                {
                    List<float> blobLocsInTask = getBlobsInImage(imgMat);
                    return blobLocsInTask;
                });


#if UNITY_EDITOR
                taskBlobDetection.ContinueWith(t2 => isBlobDetectionFinished = true);
                taskBlobDetection.ContinueWith(t2 => blobLocations = taskBlobDetection.Result);
#else
                            if (frameDepthMap != null)
                            {
                                taskBlobDetection.ContinueWith(t2 => isBlobDetectionFinished = true);
                                taskBlobDetection.ContinueWith(t2 => blobLocations = taskBlobDetection.Result);
                            }
#endif

            }


            if (isBlobDetectionFinished)
            {
                if (blobLocations.Count / 2 >= numberOfSpheresForRigidBody)
                {
#if UNITY_EDITOR // in Unity with prerecorded image and on HoloLens with real data
                    calculatePosSphere(depthValues, blobLocations);
#else
                    calculatePosSphere(frameDepthMap, blobLocations);
#endif
                    if (iscalculatedPosInCamSpace)
                    {
                        createSphereBasedOnLocAndCalcRigidBody(blobLocations);
                        iscalculatedPosInCamSpace = false;

                    }
                }
                isBlobDetectionFinished = false;
                isTrackingFinished = true;
            }



        }
    }


    void createSphereBasedOnLocAndCalcRigidBody(List<float> Locs)
    {
#if ENABLE_WINMD_SUPPORT
          poseRigidNode = researchMode.GetRigidNodePose(); // Get pose of left front camera
#endif
        if (poseRigidNode == null)// Fake data for the unity editor
        {
            poseRigidNode = new float[7] { -0.668078335060651f, 0.705903116056086f, -0.206181644810610f, 0.113407487883686f, 0, 0, 0 }; //-0.2238f, 0.1104f, -0.1021f };
        }
        Quaternion rotRigidNode = new Quaternion(poseRigidNode[0], poseRigidNode[1], poseRigidNode[2], poseRigidNode[3]);
        Vector3 posRigidNode = new Vector3(poseRigidNode[4], poseRigidNode[5], poseRigidNode[6]);
#if UNITY_EDITOR
        posRigidNode = posRigidNode + Camera.main.transform.position;
        rotRigidNode = rotRigidNode * Camera.main.transform.rotation;
#endif
        Matrix4x4 rotRigidNodeMat = Matrix4x4.Rotate(rotRigidNode);
        Matrix4x4 posRigidNodeMat = Matrix4x4.Translate(posRigidNode);

        Matrix4x4 world2DepthCam = Matrix4x4.TRS(posRigidNode, rotRigidNode, new Vector3(1, 1, 1));
        world2DepthCam = world2DepthCam * depthCameraExtrinsics.inverse;

        GameObject[] arrayOfSpheres = GameObject.FindGameObjectsWithTag("sphere");
        List<Vector3> listSpheres = new List<Vector3>();

        for (int i = 0; i < Locs.Count; i = i + 2)
        {
            Vector3 cam2point = posSpheresInCamSpaces[i / 2];
            Vector3 world2point = world2DepthCam.MultiplyPoint(cam2point);

            if (useCorrectionVector)
            {
                Vector3 directionCorrectionVector = world2point - world2DepthCam.ExtractPosition();
                directionCorrectionVector = directionCorrectionVector.normalized * 0.0065f;
                world2point = world2point + directionCorrectionVector;
                world2point.z = -world2point.z;
            }
            else
            {
                world2point.z = -world2point.z;
            }
            listSpheres.Add(world2point);
        }

        List<Vector3> orderedSpheres = new List<Vector3>();

        if (listSpheres.Count >= numberOfSpheresForRigidBody)
        {
            int[] order = getOrderOfSpheres(listSpheres);

            for (int i = 0; i < order.Length; i++)
            {
                Vector3 world2point;
                if (useKalmanFilter)
                {
                    world2point = KFspheres[i].Update(listSpheres[order[i]]);
                }
                else
                {
                    world2point = listSpheres[order[i]];
                }
                orderedSpheres.Add(world2point);
                arrayOfSpheres[i].transform.position = world2point;
            }
            Matrix4x4 poseMArker = getPoseOfMarker(orderedSpheres);
            rigidBody.transform.position = poseMArker.ExtractPosition();
            rigidBody.transform.rotation = poseMArker.ExtractRotation();
        }
    }

    public void UseCorrectionVectorOnOff()
    {
        useCorrectionVector = !useCorrectionVector;
        Debug.Log("useCorrectionVector is: " + useCorrectionVector);
    }




    int[] getOrderOfSpheres(List<Vector3> posSpheres)
    {
        int[] order = new int[numberOfSpheresForRigidBody];
        List<float[]> arrayList = new List<float[]>();
        // get all the distances from the spheres to each other
        for (int i = 0; i < posSpheres.Count; i++)
        {
            arrayList.Add(new float[posSpheres.Count]);
            for (int j = 0; j < posSpheres.Count; j++)
            {
                arrayList[i][j] = Vector3.Distance(posSpheres[i], posSpheres[j]);
            }
            Array.Sort(arrayList[i]);
        }
        // get the first (by defintion) sphere
        for (int i = 0; i < arrayList.Count; i++)
        {
            float dist1 = arrayList[i][1] - distancesSpheres[1];
            float dist2 = arrayList[i][2] - distancesSpheres[2];
            float dist3 = arrayList[i][3] - distancesSpheres[3];

            if (Math.Abs(dist1) < maxDistanceTolerance && Math.Abs(dist2) < maxDistanceTolerance && Math.Abs(dist3) < maxDistanceTolerance)
            {
                order[0] = i;
            }
        }


        // get the second, third and fourth sphere
        for (int i = 0; i < arrayList.Count; i++)
        {
            float distanceToFirstSphere = Vector3.Distance(posSpheres[order[0]], posSpheres[i]);
            if (Math.Abs(distanceToFirstSphere - distancesSpheres[1]) < maxDistanceTolerance)
            {
                order[1] = i;
            }
            else if (Math.Abs(distanceToFirstSphere - distancesSpheres[2]) < maxDistanceTolerance)
            {
                order[2] = i;
            }
            else if (Math.Abs(distanceToFirstSphere - distancesSpheres[3]) < maxDistanceTolerance)
            {
                order[3] = i;
            }
        }
        return order;
    }

    Matrix4x4 getPoseOfMarker(List<Vector3> spheresInWorld)
    {
        Matrix4x4 poseMarker = new Matrix4x4();
        List<float[]> spheresPointsWorld = new List<float[]>();
        for (int i = 0; i < spheresInWorld.Count; i++)
        {
            float[] points = new float[3];
            Vector3 p = spheresInWorld[i];
            Vector3 p_r = p;
            points[0] = p_r.x;
            points[1] = p_r.y;
            points[2] = p_r.z;
            spheresPointsWorld.Add(points);
        }
        // Do horn and get transformation
        double err;
        GeneralMatrix transf;
        horn.ComputeHorn(spheresPointsWorld, out err, out transf);
        for (int n = 0; n < 4; n++)
        {
            for (int m = 0; m < 4; m++)
            {
                poseMarker[n, m] = (float)transf.GetElement(n, m);
            }
        }
        return poseMarker;
    }


    #region Button Event Functions

    public void StartTrackingCoroutine()
    {
        Debug.Log("Start tracking .....");
        StartCoroutine(StartTrackingWithCoroutine());
    }


    public void UseKalmanFilter()
    {
        useKalmanFilter = !useKalmanFilter;
        Debug.Log("Kalman filter on: " + useKalmanFilter);
    }

    #endregion

}


public static class MatrixExtensions
{
    public static Quaternion ExtractRotation(this Matrix4x4 matrix)
    {
        Vector3 forward;
        forward.x = matrix.m02;
        forward.y = matrix.m12;
        forward.z = matrix.m22;

        Vector3 upwards;
        upwards.x = matrix.m01;
        upwards.y = matrix.m11;
        upwards.z = matrix.m21;

        return Quaternion.LookRotation(forward, upwards);
    }

    public static Vector3 ExtractPosition(this Matrix4x4 matrix)
    {
        Vector3 position;
        position.x = matrix.m03;
        position.y = matrix.m13;
        position.z = matrix.m23;
        return position;
    }

    public static Vector3 ExtractScale(this Matrix4x4 matrix)
    {
        Vector3 scale;
        scale.x = new Vector4(matrix.m00, matrix.m10, matrix.m20, matrix.m30).magnitude;
        scale.y = new Vector4(matrix.m01, matrix.m11, matrix.m21, matrix.m31).magnitude;
        scale.z = new Vector4(matrix.m02, matrix.m12, matrix.m22, matrix.m32).magnitude;
        return scale;
    }
}