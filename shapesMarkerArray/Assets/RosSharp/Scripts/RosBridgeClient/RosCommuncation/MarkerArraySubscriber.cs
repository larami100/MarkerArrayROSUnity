namespace RosSharp.RosBridgeClient
{
    public class MarkerArraySubscriber : UnitySubscriber<MessageTypes.Visualization.MarkerArray>
    {
        private MarkerArrayWriter markerArrayWriter;

        protected override void Start()
        {
            markerArrayWriter = gameObject.AddComponent<MarkerArrayWriter>();
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Visualization.MarkerArray message)
        {
            markerArrayWriter.Write(message);
        }

        private void OnEnable()
        {
            if (markerArrayWriter)
                markerArrayWriter.enabled = true;
        }

        private void OnDisable()
        {
            markerArrayWriter.enabled = false;
        }
    }
} 
