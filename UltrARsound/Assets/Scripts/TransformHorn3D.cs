using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Ultrasound.Tracking
{
    public class TransformHorn3D
    {
        // Class variables
        MarkerObject markerParams;

        public class MarkerObject
        {
            public List<float[]> markerPoints;
            public float[] cMarker;
            public List<float[]> markerN;

            public MarkerObject(List<float[]> markerPoints, float[] cMarker, List<float[]> markerN)
            {
                this.markerPoints = markerPoints; // points of reference marker geometry
                this.cMarker = cMarker; // centroid of marker geometry
                this.markerN = markerN; // "centered" marker geometry ( = points - centroid)
            }
        }

        public TransformHorn3D(List<float[]> markerPoints)
        {
            int numberOfMarkerPoints = markerPoints.Count();
            if (numberOfMarkerPoints < 3) return;

            // calculate centroid
            float[] cMarker = new float[] { 0, 0, 0 };
            for (int i = 0; i < numberOfMarkerPoints; i++)
            {
                cMarker[0] += markerPoints[i][0];
                cMarker[1] += markerPoints[i][1];
                cMarker[2] += markerPoints[i][2];
            }
            cMarker[0] /= (float)numberOfMarkerPoints;
            cMarker[1] /= (float)numberOfMarkerPoints;
            cMarker[2] /= (float)numberOfMarkerPoints;

            // Subtract point offsets, move to centroids
            List<float[]> markerN = new List<float[]>();

            for (int j = 0; j < numberOfMarkerPoints; j++)
            {
                markerN.Add(new float[] {
                    markerPoints[j][0] - cMarker[0],
                    markerPoints[j][1] - cMarker[1],
                    markerPoints[j][2] - cMarker[2] });
            }
            markerParams = new MarkerObject(markerPoints, cMarker, markerN);
        }

        public void ComputeHorn(List<float[]> points, out double error, out GeneralMatrix transform)
        {
            int numberOfMarkerPoints = markerParams.markerPoints.Count;
            error = 0;
            transform = GeneralMatrix.Identity(4, 4);

            if (numberOfMarkerPoints != points.Count) return; // Early out

            // Compute centroids
            float[] cPoints = new float[] { 0, 0, 0 };
            for (int i = 0; i < numberOfMarkerPoints; i++)
            {
                cPoints[0] += points[i][0];
                cPoints[1] += points[i][1];
                cPoints[2] += points[i][2];
            }
            cPoints[0] /= (float)points.Count;
            cPoints[1] /= (float)points.Count;
            cPoints[2] /= (float)points.Count;

            // Subtract point offsets, move to centroids
            List<float[]> pointsN = new List<float[]>();
            for (int i = 0; i < numberOfMarkerPoints; i++)
            {
                pointsN.Add(new float[] {
                    points[i][0] - cPoints[0],
                    points[i][1] - cPoints[1],
                    points[i][2] - cPoints[2] });
            }
            // Compute covariance matrix
            GeneralMatrix covMat = new GeneralMatrix(3, 3, 0);
            for (int i = 0; i < numberOfMarkerPoints; i++)
            {
                covMat += new GeneralMatrix(new double[] { markerParams.markerN[i][0], markerParams.markerN[i][1], markerParams.markerN[i][2] }, 3) *
                    (new GeneralMatrix(new double[] { pointsN[i][0], pointsN[i][1], pointsN[i][2] }, 3)).Transpose();
            }
            // Calculate SVD
            SingularValueDecomp svd = covMat.SVD();
            GeneralMatrix V = svd.GetV();
            GeneralMatrix R = V * svd.GetU().Transpose();
            // handle reflection case (det(R)=-1)   
            //if (det33(R) < 0)
            //{
            //    //GeneralMatrix V = svd.GetV();
            //    V.SetElement(0, 2, V.GetElement(0, 2) * (-1));
            //    V.SetElement(1, 2, V.GetElement(1, 2) * (-1));
            //    V.SetElement(2, 2, V.GetElement(2, 2) * (-1));
            //    R = V * svd.GetU().Transpose();
            //}
            GeneralMatrix T = R * new GeneralMatrix(new double[] { -markerParams.cMarker[0], -markerParams.cMarker[1], -markerParams.cMarker[2] }, 3) +
               new GeneralMatrix(new double[] { cPoints[0], cPoints[1], cPoints[2] }, 3);
            transform.SetMatrix(0, 2, 0, 2, R);
            transform.SetMatrix(0, 2, 3, 3, T);

            // Calculate error, centroids are already shifted
            for (int i = 0; i < numberOfMarkerPoints; i++)
            {
                GeneralMatrix dist = transform * (new GeneralMatrix(new double[] { markerParams.markerPoints[i][0], markerParams.markerPoints[i][1], markerParams.markerPoints[i][2], 1 }, 4)) -
                      new GeneralMatrix(new double[] { points[i][0], points[i][1], points[i][2], 1 }, 4);
                dist = dist.GetMatrix(0, 2, 0, 0);
                error += dist.Norm2();
            }
            error = error / numberOfMarkerPoints;
        }

        double det33(GeneralMatrix m) // Only for 3x3 matrices
        {
            return
                m.GetElement(0, 0) * m.GetElement(1, 1) * m.GetElement(2, 2) -
                m.GetElement(0, 0) * m.GetElement(1, 2) * m.GetElement(2, 1) -
                m.GetElement(0, 1) * m.GetElement(1, 0) * m.GetElement(2, 2) +
                m.GetElement(0, 1) * m.GetElement(1, 2) * m.GetElement(2, 0) +
                m.GetElement(0, 2) * m.GetElement(1, 0) * m.GetElement(2, 1) -
                m.GetElement(0, 2) * m.GetElement(1, 1) * m.GetElement(2, 0);
        }

        float distSqr(float[] p1, float[] p2)
        {
            float a = p1[0] - p2[0];
            float b = p1[1] - p2[1];
            float c = p1[2] - p2[2];
            return a * a + b * b + c * c;
        }

    }
}
