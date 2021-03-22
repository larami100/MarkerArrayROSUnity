using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class MarkerArrayWriter : MarkerWriter
    {
        private MessageTypes.Visualization.Marker[] markerArray;

        protected override void Update()
        {
            if (isReceived)
            {
                foreach (MessageTypes.Visualization.Marker marker in markerArray)
                {
                    UpdateMarker(marker);
                    markerVisualizers[marker.id].SetMarkerData(marker, Time.time);
                }
            }
            if (markerVisualizers.Count > 0)
                RemoveDeadMarker();
            isReceived = false;
        }

        public void Write(MessageTypes.Visualization.MarkerArray markerArray)
        {
            this.markerArray = markerArray.markers;
            isReceived = true;
        }
    }
} 
