using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public abstract class MarkerVisualizer : MonoBehaviour
    {
        protected bool IsNewMarkerReceived = false;

        protected GameObject markerObject;
        public MessageTypes.Visualization.Marker marker;
        protected float lastUpdateTime;

        abstract protected void Visualize();

        protected void Update()
        {
            if (!IsNewMarkerReceived)
                return;

            IsNewMarkerReceived = false;
            Visualize();
        }

        protected void OnDisable()
        {
            DestroyObject();
        }

        public virtual void DestroyObject()
        {
            Destroy(markerObject);
        }

        public void SetMarkerData(MessageTypes.Visualization.Marker marker, float time)
        {
            this.marker = marker;
            lastUpdateTime = time;
            IsNewMarkerReceived = true;
        }

        public bool IsAlive()
        {
            if ((marker.lifetime.secs != 0 || marker.lifetime.nsecs != 0) && Time.time - lastUpdateTime > marker.lifetime.secs)
                return false;
            return true;
        }
    }
} 
