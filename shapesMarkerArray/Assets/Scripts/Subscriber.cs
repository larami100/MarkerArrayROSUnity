using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using Marker = RosSharp.RosBridgeClient.MessageTypes.Visualization.Marker;
using MarkerArray = RosSharp.RosBridgeClient.MessageTypes.Visualization.MarkerArray;
using System.Linq;

public class Subscriber : MonoBehaviour
{
    public string topic;
    public string shapesName;
    public string rosConnectorGameObjectName;
    bool check_topic;
    RosSocket rosSocket;
    MarkerArray shapes;
    
    void Start () 
    {
        StartCoroutine(Init(1.0f));
    }
    
    IEnumerator Init(float delay  = 0.0f)
    {
        if (delay != 0)
           yield return new WaitForSeconds(delay);
        check_topic = false;
        rosSocket = GameObject.Find(rosConnectorGameObjectName).GetComponent<RosConnector>().RosSocket;
        if(rosSocket != null)
        {
            rosSocket.Subscribe<MarkerArray>(topic, SubscriptionHandler);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (check_topic)
        {
            foreach (GameObject item in Resources.FindObjectsOfTypeAll<GameObject>().Where(obj => obj.name == shapesName))
            {
                Destroy(item);
            } 
            
            foreach (Marker shape in shapes.markers)
            {   
                
                switch((uint)shape.type)
                {
                    case Marker.CUBE: case Marker.SPHERE: case Marker.CYLINDER:
                        VisualizeShape vs = new VisualizeShape(shape, shape.ns);
                        vs.Visualize();
                        break;
                    case Marker.MESH_RESOURCE:
                        VisualizeMeshResource va = new VisualizeMeshResource(shape, shape.ns);
                        va.Visualize();
                        break;      
                }
            }
                
            Array.Clear(shapes.markers, 0, shapes.markers.Length);
            check_topic = false;
        }
        
    }
    
    void SubscriptionHandler(MarkerArray message)
    {
        shapes = message;
        check_topic = true;
    }
   }
