/*
 * RobotVisualizer.cs
 *
 * Attach to any GameObject.  Reads joint angles sent back by ik_process.py
 * and rotates five child GameObjects to match — so you can see the
 * Unity-side robot mirror the Meshcat window in real time.
 *
 * Setup
 * ────────────────────────────────────────────────────────────────
 *  1. Create a hierarchy in your scene that matches the URDF:
 *
 *       RobotRoot
 *         └─ Joint1   (base rotation  — Y axis)
 *              └─ Joint2   (shoulder pitch — X axis)
 *                   └─ Joint3   (elbow yaw   — Z axis)
 *                        └─ Joint4   (elbow pitch — X axis)
 *                             └─ Joint5   (wrist yaw  — Z axis)
 *
 *     The exact mesh/shape doesn't matter for a test — plain cubes work.
 *
 *  2. Drag the five joint GameObjects into the `joints` array (index 0–4).
 *
 *  3. Make sure RobotIKSender is also in the scene (it opens the send socket;
 *     this script only listens on a separate port).
 *
 * What it receives
 * ────────────────────────────────────────────────────────────────
 * Listens on 127.0.0.1:10002  (must match IKConfig.ik_send_port)
 *
 *   { "joint_angles": [q0, q1, q2, q3, q4] }   ← radians, 5 joints
 *
 * Note: ik_process.py currently sends raw binary floats, not JSON.
 * This script handles both formats so you can test either way.
 */

using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class RobotVisualizer : MonoBehaviour
{
    // ── Inspector ─────────────────────────────────────────────────
    [Header("Network")]
    public int listenPort = 10002;

    [Header("Joint transforms (index 0 = joint_1 ... 4 = joint_5)")]
    public Transform[] joints = new Transform[5];

    [Header("Joint rotation axes — must match URDF")]
    // URDF axis xyz per joint:  j1=Z, j2=X, j3=Z, j4=X, j5=Z
    //
    // joint_1 origin has rpy="-1.5708 0 0", so its local Z becomes world Y.
    // joint_3/5 inherit the accumulated parent rotation — local Z also ends
    // up as world Y in the default rest pose.
    // If any joint rotates the wrong way in practice, negate that axis here.
    public Vector3[] rotationAxes = new Vector3[]
    {
        Vector3.up,      // joint_1  URDF Z, frame-rotated → world Y
        Vector3.right,   // joint_2  URDF X
        Vector3.up,      // joint_3  URDF Z, accumulated rotation → world Y
        Vector3.right,   // joint_4  URDF X
        Vector3.up,      // joint_5  URDF Z, accumulated rotation → world Y
    };

    [Header("Debug")]
    public bool logAngles = false;

    // ── Runtime ───────────────────────────────────────────────────
    private UdpClient  _udp;
    private Thread     _recvThread;
    private float[]    _latestAnglesRad = new float[5];
    private bool       _hasNewData      = false;
    private readonly object _lock       = new object();

    // ── Unity lifecycle ───────────────────────────────────────────
    void Start()
    {
        _udp = new UdpClient(listenPort);
        _recvThread = new Thread(ReceiveLoop) { IsBackground = true, Name = "IK-Recv" };
        _recvThread.Start();
        Debug.Log($"[RobotVisualizer] Listening on port {listenPort}");
    }

    void Update()
    {
        bool fresh;
        float[] angles;
        lock (_lock)
        {
            fresh  = _hasNewData;
            angles = (float[])_latestAnglesRad.Clone();
            _hasNewData = false;
        }

        if (!fresh) return;

        for (int i = 0; i < joints.Length && i < angles.Length; i++)
        {
            if (joints[i] == null) continue;
            float deg = angles[i] * Mathf.Rad2Deg;
            joints[i].localRotation = Quaternion.AngleAxis(deg, rotationAxes[i]);
        }

        if (logAngles)
        {
            string s = "";
            for (int i = 0; i < angles.Length; i++)
                s += $"J{i+1}:{angles[i]*Mathf.Rad2Deg:F1}° ";
            Debug.Log($"[RobotVisualizer] {s}");
        }
    }

    void OnDestroy()
    {
        _udp?.Close();
        _recvThread?.Interrupt();
    }

    // ── Receive thread ────────────────────────────────────────────
    void ReceiveLoop()
    {
        var ep = new IPEndPoint(IPAddress.Any, 0);
        while (true)
        {
            try
            {
                byte[] data = _udp.Receive(ref ep);
                float[] angles = TryParseBinary(data) ?? TryParseJson(data);
                if (angles != null && angles.Length >= 5)
                {
                    lock (_lock)
                    {
                        Array.Copy(angles, _latestAnglesRad,
                                   Math.Min(angles.Length, _latestAnglesRad.Length));
                        _hasNewData = true;
                    }
                }
            }
            catch (ThreadInterruptedException) { break; }
            catch (SocketException)            { break; }
            catch (Exception e)
            {
                Debug.LogWarning($"[RobotVisualizer] Recv error: {e.Message}");
            }
        }
    }

    // Binary: N × float32 little-endian  (what ik_process.py sends — 5 joints = 20 bytes)
    static float[] TryParseBinary(byte[] data)
    {
        // Must be a multiple of 4, at least 1 float, and no more than 32 joints
        if (data.Length < 4 || data.Length % 4 != 0 || data.Length > 128) return null;
        int n = data.Length / 4;
        var result = new float[n];
        for (int i = 0; i < n; i++)
            result[i] = BitConverter.ToSingle(data, i * 4);
        return result;
    }

    // JSON fallback: { "joint_angles": [f, f, f, f, f] }
    static float[] TryParseJson(byte[] data)
    {
        try
        {
            string json = Encoding.UTF8.GetString(data);
            // Minimal hand-rolled parser to avoid needing Newtonsoft
            int start = json.IndexOf('[');
            int end   = json.IndexOf(']');
            if (start < 0 || end < 0) return null;
            string[] parts = json.Substring(start + 1, end - start - 1)
                                 .Split(',');
            var result = new float[parts.Length];
            for (int i = 0; i < parts.Length; i++)
                result[i] = float.Parse(parts[i].Trim(),
                                        System.Globalization.CultureInfo.InvariantCulture);
            return result;
        }
        catch { return null; }
    }
}
