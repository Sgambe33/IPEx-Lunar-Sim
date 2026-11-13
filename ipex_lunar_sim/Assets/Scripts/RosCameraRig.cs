using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using ROS2;
using UnityEngine.Experimental.Rendering;

/// <summary>
/// Handles multiple cameras image acquisition and transmission.
/// </summary>
public class RosCameraRig : RosBehaviour
{
    private class CameraFeed
    {
        public string name;
        public Camera camera;
        public string topic;
        public string frameId;
        public double tx;
        public float interval;
        public float nextCaptureTime;
        public bool captureInFlight;
        public RenderTexture renderTexture;
        public IPublisher<sensor_msgs.msg.Image> imagePub;
        public IPublisher<sensor_msgs.msg.CameraInfo> infoPub;
        public sensor_msgs.msg.CameraInfo cameraInfo;
        public sensor_msgs.msg.Image image;
        public byte[] mono;
    }

    [Header("Stereo Cameras - Front")]
    [SerializeField] private Camera cameraFL;
    [SerializeField] private Camera cameraFR;

    [Header("Stereo Cameras - Rear")]
    [SerializeField] private Camera cameraRL;
    [SerializeField] private Camera cameraRR;

    [Header("Side Cameras")]
    [SerializeField] private Camera cameraLS;
    [SerializeField] private Camera cameraRS;

    [Header("Publish Rate")]
    [SerializeField] private float stereoHz = 10f;
    [SerializeField] private float sideHz = 2f;

    [Header("Output Settings")]
    [SerializeField] private int imageWidth = 1280;
    [SerializeField] private int imageHeight = 720;

    private readonly List<CameraFeed> feeds = new List<CameraFeed>();

    protected override void Awake()
    {
        base.Awake();

        float frontBaseline = ComputeBaseline(cameraFL, cameraFR);
        float rearBaseline = ComputeBaseline(cameraRL, cameraRR);

        AddFeed("front_left", cameraFL, "camera/front/left", "front_left_camera", 0.0, stereoHz);
        AddFeed("front_right", cameraFR, "camera/front/right", "front_right_camera", frontBaseline, stereoHz);
        AddFeed("rear_left", cameraRL, "camera/rear/left", "rear_left_camera", 0.0, stereoHz);
        AddFeed("rear_right", cameraRR, "camera/rear/right", "rear_right_camera", rearBaseline, stereoHz);
        AddFeed("side_left", cameraLS, "camera/side/left", "side_left_camera", 0.0, sideHz);
        AddFeed("side_right", cameraRS, "camera/side/right", "side_right_camera", 0.0, sideHz);
    }

    void OnDestroy()
    {
        foreach (var feed in feeds)
        {
            if (feed.renderTexture != null) feed.renderTexture.Release();
        }
    }

    static float ComputeBaseline(Camera a, Camera b)
    {
        if (a == null || b == null) return 0f;
        return Vector3.Distance(a.transform.position, b.transform.position);
    }

    void AddFeed(string name, Camera cam, string topic, string frameId, double tx, float hz)
    {
        if (cam == null) return;

        cam.enabled = false;

        feeds.Add(new CameraFeed
        {
            name = name,
            camera = cam,
            topic = topic,
            frameId = frameId,
            tx = tx,
            interval = 1f / Mathf.Max(hz, 0.01f),
            nextCaptureTime = 0f,
        });
    }

    protected override void OnRosReady()
    {
        foreach (var feed in feeds)
        {
            feed.imagePub = Advertise<sensor_msgs.msg.Image>(feed.topic + "/image_raw", RosQos.SensorData());
            feed.infoPub = Advertise<sensor_msgs.msg.CameraInfo>(feed.topic + "/camera_info", RosQos.SensorData());
            feed.renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32)
            {
                name = "RosCameraRig_" + feed.name
            };
            feed.cameraInfo = BuildCameraInfo(feed.camera, feed.tx);

            feed.mono = new byte[imageWidth * imageHeight];
            feed.image = new sensor_msgs.msg.Image
            {
                Height = (uint)imageHeight,
                Width = (uint)imageWidth,
                Encoding = "mono8",
                Is_bigendian = 0,
                Step = (uint)imageWidth,
                Data = feed.mono,
            };
            feed.image.Header.Frame_id = feed.frameId;
            feed.cameraInfo.Header = feed.image.Header;
        }
    }

    void Update()
    {
        if (!RosReady) return;

        float now = Time.time;
        foreach (var feed in feeds)
        {
            if (feed.captureInFlight || now < feed.nextCaptureTime) continue;
            feed.nextCaptureTime = now + feed.interval;
            CaptureAndPublish(feed);
        }
    }

    void CaptureAndPublish(CameraFeed feed)
    {
        feed.captureInFlight = true;
        var cam = feed.camera;

        var previousTarget = cam.targetTexture;
        cam.targetTexture = feed.renderTexture;
        cam.Render();
        cam.targetTexture = previousTarget;

        var img = feed.image;
        Stamp(img);

        int width = imageWidth;
        int height = imageHeight;

        AsyncGPUReadback.Request(feed.renderTexture, 0, GraphicsFormat.R8G8B8A8_UNorm, request =>
        {
            feed.captureInFlight = false;

            if (request.hasError)
            {
                Debug.LogWarning($"RosCameraRig: GPU readback failed for '{feed.name}'.", this);
                return;
            }
            if (!RosReady || feed.imagePub == null) return;

            ConvertToMono8(request.GetData<byte>(), feed.mono, width, height);
            feed.imagePub.Publish(img);
            feed.infoPub.Publish(feed.cameraInfo);
        });
    }

    static void ConvertToMono8(NativeArray<byte> rgba, byte[] mono, int width, int height)
    {
        int stride = width * 4; // rgba is 4 bytes per pixel

        for (int y = 0; y < height; y++)
        {
            int srcRow = (height - 1 - y) * stride; // unity image origin (0,0) is bottom-left but in ROS is top-left
            int dstRow = y * width;

            for (int x = 0; x < width; x++)
            {
                int startIdx = srcRow + x * 4;
                byte r = rgba[startIdx];
                byte g = rgba[startIdx + 1];
                byte b = rgba[startIdx + 2];
                // https://it.mathworks.com/help/matlab/ref/rgb2gray.html#buiz8mj-9 RBG->GRAY formula
                mono[dstRow + x] = (byte)(0.299f * r + 0.587f * g + 0.114f * b);
            }
        }
    }

    // https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg
    sensor_msgs.msg.CameraInfo BuildCameraInfo(Camera cam, double tx)
    {
        cam.aspect = (float)imageWidth / imageHeight;
        Matrix4x4 pm = cam.projectionMatrix;
        float fx = pm[0, 0] * imageWidth * 0.5f;
        float fy = pm[1, 1] * imageHeight * 0.5f;
        float cx = imageWidth * 0.5f * (1f - pm[0, 2]);
        float cy = imageHeight * 0.5f * (1f + pm[1, 2]);

        var info = new sensor_msgs.msg.CameraInfo();
        info.Width = (uint)imageWidth;
        info.Height = (uint)imageHeight;
        info.Distortion_model = "plumb_bob"; //ROS2 suggested parameter
        info.D = new double[5] { 0, 0, 0, 0, 0 }; //Unity's camera has no distortion

        double[] K = { fx, 0, cx, 0, fy, cy, 0, 0, 1 };
        K.CopyTo(info.K, 0);

        double[] R = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        R.CopyTo(info.R, 0);

        double[] P = { fx, 0, cx, -fx * tx, 0, fy, cy, 0, 0, 0, 1, 0 };
        P.CopyTo(info.P, 0);

        return info;
    }
}
