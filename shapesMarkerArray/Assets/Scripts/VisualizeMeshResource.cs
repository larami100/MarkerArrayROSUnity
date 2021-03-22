using UnityEngine;
using System;
using System.Linq;

namespace RosSharp.RosBridgeClient
{
    public class VisualizeMeshResource
    {
        private GameObject markerObject;
        private MessageTypes.Visualization.Marker marker;
        private string name;
        private bool isCreated = false;
        
        public VisualizeMeshResource(MessageTypes.Visualization.Marker marker, string name)
        {
            this.marker = marker;
            this.name = name;
        }

        private void Create()
        {
            markerObject = new GameObject("Mesh");
            markerObject.AddComponent<MeshFilter>();
            markerObject.AddComponent<MeshRenderer>();

            try
            {
                // Get name of mesh
                string[] s = marker.mesh_resource.Split('/');
                s = s.Last().Split('.');
                // Mesh needs a prefab in 'Assets\Resources\'
                GameObject meshObject = (GameObject)Resources.Load(s[0], typeof(GameObject));

                Mesh mesh = meshObject.GetComponentInChildren<MeshFilter>().sharedMesh;

                markerObject.GetComponent<MeshFilter>().mesh = mesh;
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }
            isCreated = true;
        }

        public void Visualize()
        {
            if (!isCreated)
                Create();
                
            markerObject.name = name;

            markerObject.transform.localPosition = TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.pose.position));
            markerObject.transform.localRotation = TransformExtensions.Ros2Unity(TypeExtensions.QuaternionMsgToQuaternion(marker.pose.orientation));

            if (!marker.frame_locked)
                markerObject.transform.parent = null;

            markerObject.transform.localScale = TransformExtensions.Ros2UnityScale(TypeExtensions.Vector3MsgToVector3(marker.scale));

            markerObject.GetComponent<MeshRenderer>().material.SetColor("_Color", TypeExtensions.ColorRGBAToColor(marker.color));
        }
    }
} 
