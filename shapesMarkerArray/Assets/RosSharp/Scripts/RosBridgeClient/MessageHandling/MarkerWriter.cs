using UnityEngine;
using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{
    public class MarkerWriter : MonoBehaviour
    {
        protected bool isReceived = false;

        protected Dictionary<int, MarkerVisualizer> markerVisualizers;

        private MessageTypes.Visualization.Marker marker;

        private void Start()
        {
            markerVisualizers = new Dictionary<int, MarkerVisualizer>();
        }

        protected virtual void Update()
        {
            if (isReceived)
            {
                UpdateMarker(marker);
                if (markerVisualizers.Count > 0)
                    markerVisualizers[marker.id].SetMarkerData(marker, Time.time);
            }

            if (markerVisualizers.Count > 0)
                RemoveDeadMarker();
            isReceived = false;
        }

        public void Write(MessageTypes.Visualization.Marker marker)
        {
            this.marker = marker;
            isReceived = true;
        }

        protected void UpdateMarker(MessageTypes.Visualization.Marker marker)
        {
            if (marker.action == MessageTypes.Visualization.Marker.DELETEALL)
            {
                foreach (int key in markerVisualizers.Keys)
                    markerVisualizers[key].DestroyObject();
            }
            if (marker.action == MessageTypes.Visualization.Marker.DELETE)
                markerVisualizers[marker.id].DestroyObject();
            else if (!markerVisualizers.ContainsKey(marker.id)
                || markerVisualizers[marker.id].marker.action != marker.action
                || markerVisualizers[marker.id].marker.type != marker.type)
            {
                switch (marker.type)
                {
                    default:
                        Debug.LogError("Marker type not available");
                        break;
                }
            }
        }

        protected void RemoveDeadMarker()
        {
            List<int> deadMarkers = new List<int>();
            foreach (int key in markerVisualizers.Keys)
            {
                if (!markerVisualizers[key].IsAlive())
                    deadMarkers.Add(key);
            }
            foreach (int key in deadMarkers)
            {
                markerVisualizers[key].DestroyObject();
                markerVisualizers.Remove(key);
            }
        }
        protected void OnDisable()
        {
            foreach (int key in markerVisualizers.Keys)
                markerVisualizers[key].DestroyObject();
            markerVisualizers.Clear();
        }
    }
} 
