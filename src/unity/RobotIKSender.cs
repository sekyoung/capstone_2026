/*
 * RobotIKSender.cs
 *
 * Attach this to any GameObject in your Unity scene.
 *
 * What it does
 * ────────────────────────────────────────────────────────────────
 * Every FixedUpdate it sends a JSON UDP packet to ik_process.py
 * describing a target hand position + elbow hint.
 *
 * Two usage modes (toggle in Inspector):
 *
 *   1. SLIDER mode  — you drag sliders in a Canvas UI to move the
 *      target around.  Good for first-contact testing with no XR rig.
 *
 *   2. TRANSFORM mode — assign any scene Transform to `handTarget`
 *      and `elbowTarget`.  Works with XR hand tracking, VR controllers,
 *      or just an empty GameObject you move in the Editor.
 *
 * How to set up (Slider mode)
 * ────────────────────────────────────────────────────────────────
 *  1. Create a Canvas → add three Sliders named X / Y / Z
 *     Range X: -0.3 → 0.3   default 0.1
 *     Range Y: -0.3 → 0.5   default 0.2
 *     Range Z:  0.0 → 0.5   default 0.15
 *  2. Assign them to the three Slider fields in this component.
 *  3. Press Play — the Python IK window will react immediately.
 *
 * How to set up (Transform mode)
 * ────────────────────────────────────────────────────────────────
 *  1. Create two empty GameObjects: "HandTarget" and "ElbowTarget"
 *  2. Drag them into `handTarget` / `elbowTarget` in the Inspector.
 *  3. Move the GameObjects in Play mode (or wire them to XR input).
 *
 * Protocol
 * ────────────────────────────────────────────────────────────────
 * Sends to  127.0.0.1:10001  (must match IKConfig.ik_recv_port)
 *
 *   { "type": "tracking",
 *     "hand_pos":  [x, y, z],
 *     "hand_rot":  [w, x, y, z],   ← identity quaternion for slider mode
 *     "elbow_pos": [x, y, z] }
 *
 * Receives from 127.0.0.1:10002 — joint angles in radians (ignored here;
 * use RobotVisualizer.cs if you want to drive a Unity robot model).
 */

using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

public class RobotIKSender : MonoBehaviour
{
    // ── Inspector fields ──────────────────────────────────────────
    [Header("Network")]
    public string pythonIP   = "127.0.0.1";
    public int    sendPort   = 10001;
    public int    recvPort   = 10002;

    [Header("Mode")]
    public bool useTransformMode = false;

    [Header("Transform mode — assign in Inspector")]
    public Transform handTarget;
    public Transform elbowTarget;

    [Header("Slider mode — assign in Inspector")]
    public Slider sliderX;
    public Slider sliderY;
    public Slider sliderZ;

    [Header("Debug")]
    public bool logPackets = false;

    // ── Private ───────────────────────────────────────────────────
    private UdpClient   _udp;
    private IPEndPoint  _sendEP;

    // current target, readable by other scripts
    [HideInInspector] public Vector3    currentHandPos;
    [HideInInspector] public Quaternion currentHandRot = Quaternion.identity;
    [HideInInspector] public Vector3    currentElbowPos;

    // ── Unity lifecycle ───────────────────────────────────────────
    void Start()
    {
        _udp    = new UdpClient();
        _sendEP = new IPEndPoint(IPAddress.Parse(pythonIP), sendPort);

        // Default elbow hint: slightly behind and below hand
        currentElbowPos = new Vector3(0.0f, 0.1f, 0.1f);

        Debug.Log($"[RobotIKSender] Sending to {pythonIP}:{sendPort}");
    }

    void FixedUpdate()
    {
        UpdateTargets();
        SendTrackingPacket();
    }

    void OnDestroy()
    {
        _udp?.Close();
    }

    // ── Target resolution ─────────────────────────────────────────
    void UpdateTargets()
    {
        if (useTransformMode)
        {
            if (handTarget  != null) currentHandPos  = handTarget.position;
            if (elbowTarget != null) currentElbowPos = elbowTarget.position;
            if (handTarget  != null) currentHandRot  = handTarget.rotation;
        }
        else
        {
            // Slider mode
            float x = sliderX != null ? sliderX.value : 0.1f;
            float y = sliderY != null ? sliderY.value : 0.2f;
            float z = sliderZ != null ? sliderZ.value : 0.15f;
            currentHandPos  = new Vector3(x, y, z);
            currentHandRot  = Quaternion.identity;
            currentElbowPos = new Vector3(x * 0.5f, y * 0.5f, z * 0.5f);
        }
    }

    // ── Packet construction & send ────────────────────────────────
    void SendTrackingPacket()
    {
        var hp  = currentHandPos;
        var hr  = currentHandRot;
        var ep  = currentElbowPos;

        // Unity uses left-hand coordinate system; Python side handles the flip.
        string json = "{"
            + $"\"type\":\"tracking\","
            + $"\"hand_pos\":[{hp.x:F4},{hp.y:F4},{hp.z:F4}],"
            + $"\"hand_rot\":[{hr.w:F4},{hr.x:F4},{hr.y:F4},{hr.z:F4}],"
            + $"\"elbow_pos\":[{ep.x:F4},{ep.y:F4},{ep.z:F4}]"
            + "}";

        byte[] data = Encoding.UTF8.GetBytes(json);
        try
        {
            _udp.Send(data, data.Length, _sendEP);
            if (logPackets) Debug.Log($"[IKSender] {json}");
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[RobotIKSender] Send failed: {e.Message}");
        }
    }

    // ── Public helpers (call from other scripts / UI buttons) ─────
    public void SetHandPosition(Vector3 pos) => currentHandPos = pos;
    public void SetElbowPosition(Vector3 pos) => currentElbowPos = pos;

    /// Call this to send a one-shot calibration packet
    public void SendCalibration(float armLengthMeters)
    {
        string json = $"{{\"type\":\"calibration\",\"arm_length\":{armLengthMeters:F4}}}";
        byte[] data = Encoding.UTF8.GetBytes(json);
        try { _udp.Send(data, data.Length, _sendEP); }
        catch (Exception e) { Debug.LogWarning($"[RobotIKSender] Calibration send failed: {e.Message}"); }
    }
}
