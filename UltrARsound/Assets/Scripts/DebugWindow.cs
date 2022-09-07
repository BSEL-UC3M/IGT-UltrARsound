using UnityEngine;

public class DebugWindow : MonoBehaviour
{
    TextMesh textMesh = new TextMesh();

    // Use this for initialization
    void Start()
    {
        textMesh = gameObject.GetComponentInChildren<TextMesh>();
    }

    void OnEnable()
    {
        Application.logMessageReceived += LogMessage;
    }

    void OnDisable()
    {
        Application.logMessageReceived -= LogMessage;
    }

    public void LogMessage(string message, string stackTrace, LogType type)
    {
        if(textMesh ==null)
        {
            textMesh = gameObject.GetComponentInChildren<TextMesh>();
        }

        if (textMesh.text.Length > 1000)
        {
            textMesh.text = message + "\n";
        }
        else
        {
            textMesh.text = message + "\n";
            //textMesh.text = textMesh.text + " \n " + message + " \n ";
        }
    }
}