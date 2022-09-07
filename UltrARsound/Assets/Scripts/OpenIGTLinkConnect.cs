using UnityEngine;
using System;
using System.Net;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Collections;
using System.Threading;
using System.Collections.Generic;

//using System.Runtime.InteropServices;

public class OpenIGTLinkConnect : MonoBehaviour
{
    //Set from config.txt, which is located in the project folder when run from the editor
    public string ipString;
    public int port = 18944;

    private SocketHandler socketForUnityAndHoloLens;

    // Information for the display image 
    public GameObject previewPlane = null;
    private Material mediaMaterial = null;
    private Texture2D mediaTexture = null;

    // Use this for initialization
    void Start()
    {
        // image to be displayed
        mediaMaterial = previewPlane.GetComponent<MeshRenderer>().material;
        mediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
        mediaMaterial.mainTexture = mediaTexture;

    }


    public void ConnectUS()
    {
        
        Debug.Log("Trying to connect to server with IP: " + ipString);
        socketForUnityAndHoloLens = new SocketHandler();
        socketForUnityAndHoloLens.Connect(ipString, port);
        StartCoroutine(ListenCoroutine());


    }

    IEnumerator ListenCoroutine()
    {
        while (true)
        {
            //Should execute only once every frame, but add additional delay if neccessary
            //yield return new WaitForSeconds(1.0f);
            yield return null;
            var byteArray = socketForUnityAndHoloLens.Listen();
            if (byteArray.Length >= 262274) // header = 130 bytes + body = 512*512 bytes
            {
                byte[] imageBytes = new byte[512 * 512];
                Buffer.BlockCopy(byteArray, 130, imageBytes, 0, 512 * 512);
                mediaTexture.LoadRawTextureData(imageBytes);
                mediaTexture.Apply();
                mediaMaterial.mainTexture = mediaTexture;
            }
        }
    }

    void OnApplicationQuit()
    {
        // Release the socket.
        if (socketForUnityAndHoloLens != null)
        {
            socketForUnityAndHoloLens.Disconnect();
        }
    }
}