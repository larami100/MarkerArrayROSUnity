namespace RosSharp.RosBridgeClient
{
    public class MarkerSubscriber : UnitySubscriber<MessageTypes.Visualization.Marker>
    {
        private MarkerWriter markerWriter;

        protected override void Start()
        {
            markerWriter = gameObject.AddComponent<MarkerWriter>();
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Visualization.Marker message)
        {
            markerWriter.Write(message);
        }

        private void OnEnable()
        {
            if (markerWriter)
                markerWriter.enabled = true;
        }

        private void OnDisable()
        {
            markerWriter.enabled = false;
        }
    }
} 
