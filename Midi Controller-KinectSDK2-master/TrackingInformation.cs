using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Ventuz.OSC;
using System.Data;
using System.Diagnostics;
using radio42.Multimedia.Midi;
using System.Windows;

namespace OSCeleton
{
    abstract public class TrackingInformation
    {
        public int sensorId;
        public abstract void Send(int pointScale, UdpWriter osc, StreamWriter fileWriter);
    }

    public class BodyTrackingInformation : TrackingInformation {
        public int user;
        public Body body;
        public bool handsOnly;
        public double time;
        public double height;
        public int ScaledLHY=0;
        public int OpenLoop = 0;
        static bool B1S = false;
        static bool B2S = false;
        static bool B3S = false;
        static bool B4S = false;
        static bool B5S = false;
        static bool B6S = false;
        static bool B7S = false;
        static bool B8S = false;
        static bool B1F = false;
        static bool B2F = false;

        private static MidiOutputDevice outDevice = new MidiOutputDevice(1);

        double head_x;
        double head_y;
        double head_z;

        double waist_x;
        double waist_y;
        double waist_z;

        double l_hand_x;
        double l_hand_y;
        double l_hand_z;

        double r_hand_x;
        double r_hand_y;
        double r_hand_z;

        double l_ankle_x;
        double l_ankle_y;
        double l_ankle_z;
        double r_ankle_x;
        double r_ankle_y;
        double r_ankle_z;


        public static List<String> oscMapping = new List<String> { "",
            "head", "neck", "torso", "waist",
            "l_collar", "l_shoulder", "l_elbow", "l_wrist", "l_hand", "l_fingertip",
            "r_collar", "r_shoulder", "r_elbow", "r_wrist", "r_hand", "r_fingertip",
            "l_hip", "l_knee", "l_ankle", "l_foot",
            "r_hip", "r_knee", "r_ankle", "r_foot" };



        public BodyTrackingInformation(int sensorId, int user, Body body, bool handsOnly, double time)
        {
            this.sensorId = sensorId;
            this.user = user;
            this.body = body;
            this.handsOnly = handsOnly;
            this.time = time;
        }

        public static class Scaling
        {
            public static int MapRangeMIDI(double oldMin, double oldMax, int newMin, int newMax, double x)
            {
                if (x<oldMin) {
                    x = oldMin;

                }

                if (x>oldMax) {
                    x = oldMax;
                }
                return (int)(newMin + ((x - oldMin) * (newMax - newMin) / (oldMax - oldMin)));
            }

        }

    public override void Send(int pointScale, UdpWriter osc, StreamWriter fileWriter)
        {
            if (OpenLoop==0)
            {

                outDevice.Open();
                OpenLoop = 1;
            }
            try
            {
                height = (body.Joints[JointType.Head].Position.Y - body.Joints[JointType.SpineBase].Position.Y) / 0.75;
            }
            catch
            {
                height =0;
            }
            if (body == null) return;
            if (body.Joints == null) return;
            if (body.Joints.Count < 20) return;
            if (body.JointOrientations == null) return;
            if (body.JointOrientations.Count < 20) return;
            if (!body.IsTracked) return;
            try
            {
                if (handsOnly)
                {
                    ProcessHandStateInformation(1, body.Joints[JointType.HandLeft], body.JointOrientations[JointType.HandLeft], body.HandLeftState, body.HandLeftConfidence, time, pointScale, osc, fileWriter);
                    ProcessHandStateInformation(2, body.Joints[JointType.HandRight], body.JointOrientations[JointType.HandRight], body.HandRightState, body.HandRightConfidence, time, pointScale, osc, fileWriter);
                }
                else
                {
                    head_x = body.Joints[JointType.Head].Position.X * height;
                    head_y = body.Joints[JointType.Head].Position.Y * height;
                    head_z = body.Joints[JointType.Head].Position.Z * height;

                    waist_x = body.Joints[JointType.SpineBase].Position.X * height;
                    waist_y = body.Joints[JointType.SpineBase].Position.Y * height;
                    waist_z = (body.Joints[JointType.Head].Position.Z - body.Joints[JointType.SpineBase].Position.Z) * height;

                    l_hand_x = body.Joints[JointType.HandLeft].Position.X * height;
                    l_hand_y = body.Joints[JointType.HandLeft].Position.Y * height;
                    l_hand_z = (body.Joints[JointType.Head].Position.Z - body.Joints[JointType.HandLeft].Position.Z) * height;

                    r_hand_x = body.Joints[JointType.HandRight].Position.X * height;
                    r_hand_y = body.Joints[JointType.HandRight].Position.Y * height;
                    r_hand_z = (body.Joints[JointType.Head].Position.Z - body.Joints[JointType.HandRight].Position.Z) * height;

                    l_ankle_x = body.Joints[JointType.AnkleLeft].Position.X * height;
                    l_ankle_y = body.Joints[JointType.AnkleLeft].Position.Y * height;
                    l_ankle_z = (body.Joints[JointType.Head].Position.Z - body.Joints[JointType.AnkleLeft].Position.Z) * height;

                    r_ankle_x = body.Joints[JointType.AnkleRight].Position.X * height;
                    r_ankle_y = body.Joints[JointType.AnkleRight].Position.Y * height;
                    r_ankle_z = (body.Joints[JointType.Head].Position.Z - body.Joints[JointType.AnkleRight].Position.Z) * height;

                    ProcessJointInformation(1, body.Joints[JointType.Head], body.JointOrientations[JointType.Head], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(2, body.Joints[JointType.SpineShoulder], body.JointOrientations[JointType.SpineShoulder], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(3, body.Joints[JointType.SpineMid], body.JointOrientations[JointType.SpineMid], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(4, body.Joints[JointType.SpineBase], body.JointOrientations[JointType.SpineBase], time, pointScale, osc, fileWriter);
                    // ProcessJointInformation(5, body.Joints[JointType.], body.JointOrientations[JointType.], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(6, body.Joints[JointType.ShoulderLeft], body.JointOrientations[JointType.ShoulderLeft], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(7, body.Joints[JointType.ElbowLeft], body.JointOrientations[JointType.ElbowLeft], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(8, body.Joints[JointType.WristLeft], body.JointOrientations[JointType.WristLeft], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(9, body.Joints[JointType.HandLeft], body.JointOrientations[JointType.HandLeft], time, pointScale, osc, fileWriter);
                    // ProcessJointInformation(10, body.Joints[JointType.], body.JointOrientations[JointType.], time, pointScale, osc, fileWriter);
                    // ProcessJointInformation(11, body.Joints[JointType.], body.JointOrientations[JointType.], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(12, body.Joints[JointType.ShoulderRight], body.JointOrientations[JointType.ShoulderRight], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(13, body.Joints[JointType.ElbowRight], body.JointOrientations[JointType.ElbowRight], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(14, body.Joints[JointType.WristRight], body.JointOrientations[JointType.WristRight], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(15, body.Joints[JointType.HandRight], body.JointOrientations[JointType.HandRight], time, pointScale, osc, fileWriter);
                    // ProcessJointInformation(16, body.Joints[JointType.], body.JointOrientations[JointType.], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(17, body.Joints[JointType.HipLeft], body.JointOrientations[JointType.HipLeft], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(18, body.Joints[JointType.KneeLeft], body.JointOrientations[JointType.KneeLeft], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(19, body.Joints[JointType.AnkleLeft], body.JointOrientations[JointType.AnkleLeft], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(20, body.Joints[JointType.FootLeft], body.JointOrientations[JointType.FootLeft], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(21, body.Joints[JointType.HipRight], body.JointOrientations[JointType.HipRight], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(22, body.Joints[JointType.KneeRight], body.JointOrientations[JointType.KneeRight], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(23, body.Joints[JointType.AnkleRight], body.JointOrientations[JointType.AnkleRight], time, pointScale, osc, fileWriter);
                    ProcessJointInformation(24, body.Joints[JointType.FootRight], body.JointOrientations[JointType.FootRight], time, pointScale, osc, fileWriter);

                    ProcessHandStateInformation(1, body.Joints[JointType.HandLeft], body.JointOrientations[JointType.HandLeft], body.HandLeftState, body.HandLeftConfidence, time, pointScale, osc, fileWriter);
                    ProcessHandStateInformation(2, body.Joints[JointType.HandRight], body.JointOrientations[JointType.HandRight], body.HandRightState, body.HandRightConfidence, time, pointScale, osc, fileWriter);
                }
            } catch (NullReferenceException ex) {
                // Happens sometimes. Probably because we should copy body before processing it in another thread.
                Console.WriteLine(ex.Message);
            }

            //Cuadricula de ocho botones con la mano derecha
            //Primer boton
            if (r_hand_x < 0.12 && r_hand_y > 0.62 && r_hand_z >= 0.4 && B1S == false)
            {
                B1S = true;
                MidiShortMessage B1RH = new MidiShortMessage(MIDIStatus.NoteOn, 9, 60, 64, 6);
                outDevice.Send(B1RH);
                Debug.WriteLine("Activated B1");
            }

            else if (r_hand_z < 0.4 && B1S == true)
            {
                B1S = false;
                outDevice.Send(MIDIStatus.NoteOff, 9, 60, 64);
            }
            Debug.WriteLine(r_hand_z);
            //Segundo boton
            if (r_hand_x > 0.15 && r_hand_y > 0.62 && r_hand_z > 0.4 && B2S == false)
            {
                B2S = true;
                MidiShortMessage B2RH = new MidiShortMessage(MIDIStatus.NoteOn, 9, 61, 64, 6);
                outDevice.Send(B2RH);
            }
            else if (r_hand_z < 0.4 && B2S == true)
            {
                B2S = false;
                outDevice.Send(MIDIStatus.NoteOff, 9, 61, 64);
            }

            //Tercer boton
            if (r_hand_x < 0.12 && 0.58 > r_hand_y && r_hand_y > 0.32 && r_hand_z > 0.4 && B3S == false)
            {
                B3S = true;
                MidiShortMessage B3RH = new MidiShortMessage(MIDIStatus.NoteOn, 9, 62, 64, 6);
                outDevice.Send(B3RH);
            }
            else if (r_hand_z < 0.4 && B3S == true)
            {
                B3S = false;
                outDevice.Send(MIDIStatus.NoteOff, 9, 62, 64);
            }

            //Cuarto boton
            if (r_hand_x > 0.15 && 0.58 > r_hand_y && r_hand_y > 0.32 && r_hand_z > 0.4 && B4S == false)
            {
                MidiShortMessage B4RH = new MidiShortMessage(MIDIStatus.NoteOn, 9, 63, 64, 6);
                outDevice.Send(B4RH);
                B4S = true;
            }
            else if (r_hand_z < 0.4 && B4S == true)
            {
                B4S = false;
                outDevice.Send(MIDIStatus.NoteOff, 9, 63, 64);
            }

            //Quinto boton
            if (r_hand_x < 0.12 && 0.28 > r_hand_y && r_hand_z > 0.4 && B5S == false)
            {
                MidiShortMessage B5RH = new MidiShortMessage(MIDIStatus.NoteOn, 9, 64, 64, 6);
                outDevice.Send(B5RH);
                B5S = true;
            }
            else if (r_hand_z < 0.4 && B5S == true)
            {
                outDevice.Send(MIDIStatus.NoteOff, 9, 64, 64);
                B5S = false;
            }

            //Sexto boton
            if (r_hand_x > 0.15 && 0.28 > r_hand_y && r_hand_z > 0.4 && B6S == false)
            {
                MidiShortMessage B6RH = new MidiShortMessage(MIDIStatus.NoteOn, 9, 65, 64, 6);
                outDevice.Send(B6RH);
                B6S = true;
            }
            else if (r_hand_z < 0.4 && B6S == true)
            {
                outDevice.Send(MIDIStatus.NoteOff, 9, 65, 64);
                B6S = false;
            }


            //Los botones de los pies

            //El pie izquierdo
            if (l_ankle_z > 0.08 && B1F == false)
            {
                MidiShortMessage B9LA = new MidiShortMessage(MIDIStatus.NoteOn, 9, 68, 64, 6);
                outDevice.Send(B9LA);
                B1F = true;
            }
            else if (l_ankle_z < 0.08 && B1F == true)
            {
                outDevice.Send(MIDIStatus.NoteOff, 9, 68, 64);
                B1F = false;
            }

            //El pie derecho
            if (r_ankle_z > 0.08 && B2F == false)
            {
                MidiShortMessage B9RA = new MidiShortMessage(MIDIStatus.NoteOn, 9, 69, 64, 6);
                outDevice.Send(B9RA);
                B2F = true;
            }
            else if (r_ankle_z < 0.08 && B2F == true)
            {
                outDevice.Send(MIDIStatus.NoteOff, 9, 68, 64);
                B2F = false;
            }

        }

        double JointToConfidenceValue(Joint j)
        {
            if (j.TrackingState == TrackingState.Tracked) return 1;
            if (j.TrackingState == TrackingState.Inferred) return 0.5;
            if (j.TrackingState == TrackingState.NotTracked) return 0.1;
            return 0.5;
        }
 
        void ProcessJointInformation(int joint, Joint j, JointOrientation jo, double time, int pointScale, UdpWriter osc, StreamWriter fileWriter)
        {
            if (j == null) return;
            if (jo == null) return;
            SendJointMessage(joint,
                j.Position.X, j.Position.Y, j.Position.Z,
                JointToConfidenceValue(j), time,
                pointScale, osc, fileWriter);
            SendMIDIMessage();
        }

        int HandStateToValue(HandState hs)
        {
            if (hs == HandState.Open) return 1;
            if (hs == HandState.Closed) return 2;
            if (hs == HandState.Lasso) return 3;
            if (hs == HandState.Unknown) return 4;
            if (hs == HandState.NotTracked) return 5;
            return 6;
        }

        double TrackingConfidenceToValue(TrackingConfidence c)
        { 
            if (c == TrackingConfidence.High) return 1;
            if (c == TrackingConfidence.Low) return 0;
            return 0.5;
        }

        void ProcessHandStateInformation(int joint, Joint j, JointOrientation jo, HandState state, TrackingConfidence confidence, double time, int pointScale, UdpWriter osc, StreamWriter fileWriter)
        {
            SendHandStateMessage(joint,
                j.Position.X, j.Position.Y, j.Position.Z,
                JointToConfidenceValue(j),
                HandStateToValue(state),
                TrackingConfidenceToValue(confidence), time,
                pointScale, osc, fileWriter);
        }

        public void SendMIDIMessage()
        {
            //Knobs de la mano izquierda
            //Knob en y
            int ScaledLHY = Scaling.MapRangeMIDI(-0.1, 0.8, 0, 127, l_hand_y);
            MidiShortMessage KnobLHY = new MidiShortMessage(MIDIStatus.ControlChange, 5, 1, (byte)ScaledLHY, 3);
            outDevice.Send(KnobLHY);

            //Knob en x
            int ScaledLHX = Scaling.MapRangeMIDI(-0.5, -0.1, 0, 127, l_hand_x);
            MidiShortMessage KnobLHX = new MidiShortMessage(MIDIStatus.ControlChange, 6, 1, (byte)ScaledLHX, 4);
            outDevice.Send(KnobLHX);

            //Knob en z
            int ScaledLHZ = Scaling.MapRangeMIDI(-0.1, 0.4, 0, 127, l_hand_z);
            MidiShortMessage KnobLHZ = new MidiShortMessage(MIDIStatus.ControlChange, 7, 1, (byte)ScaledLHZ, 5);
            outDevice.Send(KnobLHZ);
        }


        void SendJointMessage(int joint, double x, double y, double z, double confidence, double time, int pointScale, UdpWriter osc, StreamWriter fileWriter)
        {
            if (osc != null)
            {
                osc.Send(new OscElement("/osceleton2/joint/"+oscMapping[joint], (float)(x * pointScale), (float)(-y * pointScale), (float)(z*pointScale)));
            }
            if (fileWriter != null)
            {
                // Joint, user, joint, x, y, z, confidence, time
                fileWriter.WriteLine("Joint," + sensorId + "," + user + "," + joint + "," +
                    (x * pointScale).ToString().Replace(",", ".") + "," +
                    (-y * pointScale).ToString().Replace(",", ".") + "," +
                    (z * pointScale).ToString().Replace(",", ".") + "," +
                    confidence.ToString().Replace(",", ".") + "," +
                    time.ToString().Replace(",", "."));
            }
        }

        void SendHandStateMessage(int hand, double x, double y, double z, double confidence, int state, double stateConfidence, double time, int pointScale, UdpWriter osc, StreamWriter fileWriter)
        {
            if (osc != null)
            {
                string address_h = "/osceleton2/hand/" + hand.ToString();
                osc.Send(new OscElement(address_h,
                    (float)(x * pointScale), (float)(-y * pointScale), (float)(z * pointScale)));
            }
            if (fileWriter != null)
            {
                // Hand, user, joint, x, y, z, confidence, state, stateConfidence, time
                fileWriter.WriteLine("Hand," + sensorId + "," + user + "," + hand + "," +
                    (x * pointScale).ToString().Replace(",", ".") + "," +
                    (-y * pointScale).ToString().Replace(",", ".") + "," +
                    (z * pointScale).ToString().Replace(",", ".") + "," +
                    confidence.ToString().Replace(",", ".") + "," +
                    state + "," +
                    stateConfidence.ToString().Replace(",", ".") + "," +
                    time.ToString().Replace(",", "."));
            }
        }
    }

    public class FaceRotationTrackingInformation : TrackingInformation {
        public int user;
        public int pitch, yaw, roll;
        public double time;

        public FaceRotationTrackingInformation(int sensorId, int user, int pitch, int yaw, int roll, double time)
        {
            this.sensorId = sensorId;
            this.user = user;
            this.pitch = pitch;
            this.yaw = yaw;
            this.roll = roll;
            this.time = time;
        }

        public override void Send(int pointScale, UdpWriter osc, StreamWriter fileWriter)
        {
            if (osc != null)
            {
                osc.Send(new OscElement(
                    "/osceleton2/face_rotation",
                    sensorId, user,
                    pitch, yaw, roll,
                    time));
            }
            if (fileWriter != null)
            {
                fileWriter.WriteLine("FaceRotation," +
                    sensorId + "," + user + "," +
                    pitch + "," + yaw + "," + roll + "," +
                    time.ToString().Replace(",", "."));
            }
        }
    }
    
    public class FacePropertyTrackingInformation : TrackingInformation {
        int user;
        float happy, engaged, wearingGlasses, leftEyeClosed, rightEyeClosed, mouthOpen, mouthMoved, lookingAway;
        double time;
        public FacePropertyTrackingInformation(int sensorId, int user, float happy, float engaged, float wearingGlasses,
            float leftEyeClosed, float rightEyeClosed, float mouthOpen, float mouthMoved, float lookingAway, double time) {
            this.sensorId = sensorId;
            this.user = user;
            this.happy = happy;
            this.engaged = engaged;
            this.wearingGlasses = wearingGlasses;
            this.leftEyeClosed = leftEyeClosed;
            this.rightEyeClosed = rightEyeClosed;
            this.mouthOpen = mouthOpen;
            this.mouthMoved = mouthMoved;
            this.lookingAway = lookingAway;
            this.time = time;
        }
        
            public override void Send(int pointScale, UdpWriter osc, StreamWriter fileWriter)
            {
                if (osc != null)
                {
                    osc.Send(new OscElement(
                        "/osceleton2/face_property",
                        sensorId, user,
                        happy, engaged, wearingGlasses, leftEyeClosed, rightEyeClosed, mouthOpen, mouthMoved, lookingAway,
                        time));
                }
                if (fileWriter != null)
                {
                    fileWriter.WriteLine("FaceProperty," +
                        sensorId + "," + user + "," +
                        happy.ToString().Replace(",", ".") + "," +
                        engaged.ToString().Replace(",", ".") + "," +
                        wearingGlasses.ToString().Replace(",", ".") + "," +
                        leftEyeClosed.ToString().Replace(",", ".") + "," +
                        rightEyeClosed.ToString().Replace(",", ".") + "," +
                        mouthOpen.ToString().Replace(",", ".") + "," +
                        mouthMoved.ToString().Replace(",", ".") + "," +
                        lookingAway.ToString().Replace(",", ".") + "," +
                        time.ToString().Replace(",", "."));
                }
            }
    }
}
