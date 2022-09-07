using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.VideoModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.UnityUtils;
using System;

public class KalmanFilterSphere
{

    public KalmanFilter KF;
    public System.DateTime t_minusOne;
    System.DateTime t_zero;
    float delta_t = 1;
    int[] imSize = new int[2] { 512, 512 };
    public Vector3 xyz_estimated;
    Mat transitionMat = new Mat(10, 10, CvType.CV_32F);
    Mat measurement;


    /// <summary>
    /// Constructor to create a Kalman Filter.
    /// </summary>
    public KalmanFilterSphere()
    {

        KF = new KalmanFilter(9, 3, 0, CvType.CV_32FC1);

        // intialization of KF...
        transitionMat = new Mat(9, 9, CvType.CV_32F);
        transitionMat.put(0, 0, new float[] {
            1, 0, 0, delta_t,0, 0, 0.5f*delta_t*delta_t, 0, 0,
            0, 1, 0, 0, delta_t,0, 0, 0.5f*delta_t*delta_t, 0,
            0, 0, 1, 0, 0, delta_t,0, 0, 0.5f*delta_t*delta_t,
            0, 0, 0, 1, 0, 0, delta_t, 0, 0,
            0, 0, 0, 0, 1, 0, 0, delta_t, 0,
            0, 0, 0, 0, 0, 1, 0, 0, delta_t,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1
        });


        KF.set_transitionMatrix(transitionMat);

        measurement = new Mat(3, 1, CvType.CV_32FC1);
        measurement.setTo(Scalar.all(0));



        //// Set initial state estimate.
        Mat statePreMat = KF.get_statePre();
        statePreMat.put(0, 0, new float[] { 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        Mat statePostMat = KF.get_statePost();
        statePostMat.put(0, 0, new float[] { 0, 0, 0, 0, 0, 0, 0, 0, 0 });

        Mat measurementMat = new Mat(3, 9, CvType.CV_32FC1); //???????????????
        Core.setIdentity(measurementMat);
        KF.set_measurementMatrix(measurementMat);

        Mat processNoiseCovMat = new Mat(9, 9, CvType.CV_32FC1);
        Core.setIdentity(processNoiseCovMat, Scalar.all(1e-4));
        KF.set_processNoiseCov(processNoiseCovMat);

        Mat measurementNoiseCovMat = new Mat(3, 3, CvType.CV_32FC1);
        Core.setIdentity(measurementNoiseCovMat, Scalar.all(0.001));
        KF.set_measurementNoiseCov(measurementNoiseCovMat);

        Mat errorCovPostMat = new Mat(9, 9, CvType.CV_32FC1);
        Core.setIdentity(errorCovPostMat, Scalar.all(.1));
        KF.set_errorCovPost(errorCovPostMat);


        t_minusOne = System.DateTime.UtcNow;

    }


    public Vector3 Update(Vector3 xyz_measured)
    {


        System.DateTime t_zero = System.DateTime.UtcNow;
        System.TimeSpan t = t_zero - this.t_minusOne;
        this.delta_t = (float)t.TotalSeconds;

        transitionMat.put(0, 0, new float[] {
            1, 0, 0, delta_t,0, 0, 0.5f*delta_t*delta_t, 0, 0,
            0, 1, 0, 0, delta_t,0, 0, 0.5f*delta_t*delta_t, 0,
            0, 0, 1, 0, 0, delta_t,0, 0, 0.5f*delta_t*delta_t,
            0, 0, 0, 1, 0, 0, delta_t, 0, 0,
            0, 0, 0, 0, 1, 0, 0, delta_t, 0,
            0, 0, 0, 0, 0, 1, 0, 0, delta_t,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1
        });


        double[] predictedState = new double[9];
        double[] estimatedState;

        // TODO: Does the state transition matrix get updated automatically?
        // No it doesn't --> fill in for each new cycle



        // First predict, to update the internal statePre variable.
        using (Mat prediction = KF.predict())
        {
            for (int i = 0; i < 9; i++)
            {
                predictedState[i] = prediction.get(i, 0)[0];

            }
        }


        // Do measurement prediction
        // Update measurement mat


        Mat measurementMat = new Mat(3, 9, CvType.CV_32FC1); //???????????????
        measurementMat.put(0, 0, new float[] { 1, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 1, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 1, 0, 0, 0, 0, 0, 0});
        KF.set_measurementMatrix(measurementMat);


        // Set the true measurement

        measurement.put(0, 0, new float[] { xyz_measured.x, xyz_measured.y, xyz_measured.z });

        // The update phase.
        using (Mat estimated = KF.correct(measurement))
        {
            xyz_estimated = new Vector3((float)estimated.get(0, 0)[0], (float)estimated.get(1, 0)[0], (float)estimated.get(2, 0)[0]);
        }

        this.t_minusOne = System.DateTime.UtcNow;

        return xyz_estimated;
    }

}