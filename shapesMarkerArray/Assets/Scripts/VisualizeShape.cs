using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class VisualizeShape
    {
        private bool isCreated = false;
        private GameObject markerObject;
        private MessageTypes.Visualization.Marker marker;
        private string name;
        
        public VisualizeShape(MessageTypes.Visualization.Marker marker, string name)
        {
            this.marker = marker;
            this.name = name;
        }   

        private void Create()
        {
            switch (marker.type)
            {
                case (int)MessageTypes.Visualization.Marker.CUBE:
                    markerObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    break;
                case (int)MessageTypes.Visualization.Marker.SPHERE:
                    markerObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    break;
                case (int)MessageTypes.Visualization.Marker.CYLINDER:
                    markerObject = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                    break;
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

            markerObject.transform.localScale = TransformExtensions.Ros2UnityScale(TypeExtensions.Vector3MsgToVector3(marker.scale));
            if (marker.type == MessageTypes.Visualization.Marker.CYLINDER)
                markerObject.transform.localScale = new Vector3(markerObject.transform.localScale.x, markerObject.transform.localScale.y / 2, markerObject.transform.localScale.z);

            markerObject.GetComponent<Renderer>().material.SetColor("_Color", TypeExtensions.ColorRGBAToColor(marker.color));

            if (!marker.frame_locked)
                markerObject.transform.parent = null;
        }
    }
} 
