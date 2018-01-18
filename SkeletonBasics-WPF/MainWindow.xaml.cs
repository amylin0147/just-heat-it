//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Timers;
    using System;
    using System.Media;
    using Microsoft.Kinect;
    using System.IO.Ports;

    /// Interaction logic for MainWindow.xaml
    public partial class MainWindow : Window
    {
        /// Width of output drawing
        private const float RenderWidth = 640.0f;
        /// Height of our output drawing
        private const float RenderHeight = 480.0f;
        /// Thickness of drawn joint lines
        private const double JointThickness = 5;//3;
        /// Thickness of body center ellipse
        private const double BodyCenterThickness = 10;
        /// Thickness of clip edge rectangles
        private const double ClipBoundsThickness = 10;
        /// Brush used to draw skeleton center point
        private readonly Brush centerPointBrush = Brushes.Blue;
        /// Brush used for drawing joints that are currently tracked
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        /// Brush used for drawing joints that are currently inferred
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        /// Pen used for drawing bones that are currently tracked
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 16);//6);
        /// Pen used for drawing bones that are currently inferred
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 16);//1);
        /// Active Kinect sensor
        private KinectSensor sensor;
        /// Drawing group for skeleton rendering output
        private DrawingGroup drawingGroup;
        /// Drawing image that we will display
        private DrawingImage imageSource;

        /*** GLOBALS ADDED BY AMY ***/

        /// dance move prompt
        System.Windows.Media.Imaging.BitmapImage danceImage;
        private int danceState;
        enum danceStateOptions {JumpingJack, ArmCircle, Disco, Cabbage, Floss, Balloons};
        private int timePerDanceMove = 5000; //milliseconds
        private static System.Timers.Timer aTimer;
        // time when the current dance move started
        private DateTime GracePeriodEndTime = new DateTime(0);

        //communication with arduino SERIAL 
        SerialPort port;
        int serialDebugVar; //for serial demo/debug
        enum serialMessageOptions {doNotUse, CorrectMove, IncorrectMove};

        int countdown = 3;

        //music
        SoundPlayer player;

        //debug globals
        private bool ARDUINO_CONNECTED = true;

        /*** GLOBALS ADDED BY IRENE ***/

        /// global index counter 
        private int index = 0;
        private int lives = 3;
        //private bool endOfRow = false;
        private const int ARRLEN = 50;
        private const int Xcoord = 0;
        private const int Ycoord = 1;
        private const int Zcoord = 2;
        private const int SUMcoord = 0;
        private const int SOScoord = 1;
        private const int VARcoord = 2;
        /// 2D array for (x,y,z) coordinates; 2 rows, 10 cols
        private double[,] HandRight_arr = new double[3, ARRLEN];
        private double[,] HandLeft_arr = new double[3, ARRLEN];
        private double[,] KneeRight_arr = new double[3, ARRLEN];
        private double[,] KneeLeft_arr = new double[3, ARRLEN];
        /// (SUM/SOS/VAR , X/Y/Z)
        private double[,] HandRightStats_arr = new double[3,3];
        private double[,] HandLeftStats_arr = new double[3,3];
        private double[,] KneeRightStats_arr = new double[3,3];
        private double[,] KneeLeftStats_arr = new double[3,3];

        //variance thresholds
        private const double JJ_HANDTHRESH = .0001;     //variance lower bound
        private const double JJ_KNEETHRESH = .0000001;  //variance lower bound
        private const double DSC_HRTHRESH = .01;      //variance lower bound
        private const double DSC_HLTHRESH = .3;         //dist bw left hand and lef hip upper bound
        private const double STRAIGHT_DELTA = 2;
        private const double AC_YHANDTHRESH = .001;    //variance for vertical lower bound
        private const double AC_XHANDTHRESH = .01;    //variance for horizontal upper bound
        private const double CP_XHANDTHRESH = .007;    //variance lower bound
        private const double CP_YHANDTHRESH = .007;    //variance upper bound
        private const double CP_ZHANDTHRESH = .01;    //variance lower bound

        /// Initializes a new instance of the MainWindow class.
        public MainWindow()
        {
            InitializeComponent();
        }

        /// Draws indicators to show which edges are clipping skeleton data
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// Execute startup tasks
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource; 

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady; 

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            this.danceState = 0;

            //serial
            
            if(ARDUINO_CONNECTED) arduinoSetup();

            var path = @"C:\Users\amyli\just-heat-it\SkeletonBasics-WPF\Song\blurredlines.wav";
            player = new SoundPlayer(path);
            player.Load();

        }

        /// Execute shutdown tasks
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }

            // Close the port
            if(port != null) port.Close();
        }

        /// Return distance in meters between two joints
        private double GetJointDistance(Skeleton skeleton, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return -1;
            }

            double dist =   Math.Sqrt(
                            Math.Pow((joint0.Position.X - joint1.Position.X), 2) +
                            Math.Pow((joint0.Position.Y - joint1.Position.Y), 2) +
                            Math.Pow((joint0.Position.Z - joint1.Position.Z), 2)
                            );
            return dist;
        }

        /// Return position of a joint
        private double GetJointPosition(Skeleton skeleton, JointType jointType, int coord)
        {
            Joint j = skeleton.Joints[jointType];
            // If we can't find the joint, exit
            if (j.TrackingState == JointTrackingState.NotTracked) return -10;
            
            double position = 0;
            switch (coord)
            {
                case Xcoord: 
                    position = j.Position.X;
                    break;
                case Ycoord: 
                    position = j.Position.Y;
                    break;
                case Zcoord: 
                    position = j.Position.Z;
                    break;
            }
            return position;
        }

        /// Return sum of array
        private double GetArrSum(double[,] arr, int axis)
        {
            double sum = 0;
            for (int i = 0; i < ARRLEN; i++)
            {
                sum += arr[axis, i];
            }
            return sum;
        }

        /// Return sum of squares of array
        private double GetArrSOS(double[,] arr, int axis, double avg)
        {
            double sos = 0;
            for (int i = 0; i < ARRLEN; i++)
            {
                sos += Math.Pow((arr[axis, i] - avg), 2.0);
            }
            return sos;
        }

        /// Return variance of array
        private double GetVariance(double SoS, int len)
        {
            return SoS / (double) (len - 1);
        }

        /// Update coordinate, sum, sum of squares, and variance of a joint
        private void LogJointPosition(Skeleton skeleton, JointType jointType, 
                                        double[,] arr, double[,] arrConst, int index, int coord)
        {
            Joint j = skeleton.Joints[jointType];
            // If we can't find the joint, exit
            if (j.TrackingState == JointTrackingState.NotTracked) return;
            //update array of coordinates
            double position = this.GetJointPosition(skeleton, jointType, coord);
            // if joint position was an error, copy the previous cell
            if (position == -10) arr[coord,index] = arr[coord,(index-1)%ARRLEN];
        }

        // compute sum, sum of squares, and variance of a joint
        private void ComputeStats(double[,] arr, double[,] arrConst, int axis)
        {
            //update sum
            arrConst[SUMcoord, axis] = this.GetArrSum(arr, axis);
            //update sum of squares
            double avg = arrConst[SUMcoord, axis] / (double)ARRLEN;
            arrConst[SOScoord, axis] = this.GetArrSOS(arr, axis, avg);
            //update variance
            arrConst[VARcoord, axis] = this.GetVariance(arrConst[SOScoord, axis], ARRLEN);
        }

        private double GetSlope(double x1, double x2, double y1, double y2)
        {
            return (y1 - y2) / (x1 - x2);
        }

        /// Return true if slope from elbow to wrist is near slope from shoulder to elbow
        private bool isArmStraight(Skeleton skeleton, JointType wrist, JointType elbow, JointType shoulder)
        {
            double wristX = GetJointPosition(skeleton, wrist, Xcoord);
            double wristY = GetJointPosition(skeleton, wrist, Ycoord);
            double elbowX = GetJointPosition(skeleton, elbow, Xcoord);
            double elbowY = GetJointPosition(skeleton, elbow, Ycoord);
            double shoulderX = GetJointPosition(skeleton, shoulder, Xcoord);
            double shoulderY = GetJointPosition(skeleton, shoulder, Ycoord);

            if (wristX == -10 || wristY == -10 || elbowX == -10 || elbowY == -10 || shoulderX == -10 || shoulderY == -10)
            {
                // one of the joints returned an error
                return false;
            }

            double slope1 = GetSlope(wristX, elbowX, wristY, elbowY);
            double slope2 = GetSlope(elbowX, shoulderX, elbowY, shoulderY);

            System.Console.WriteLine("isArmStraight\t" + System.Math.Abs(slope1 - slope2));

            return System.Math.Abs(slope1 - slope2) < STRAIGHT_DELTA;
        }

        // return true if there is enough variance in hands and knees
        private bool isJumpingJack()
        {
            //update sum, sum of squares, and variance
            //hands
            this.ComputeStats(HandRight_arr, HandRightStats_arr, Xcoord);
            this.ComputeStats(HandLeft_arr, HandLeftStats_arr, Xcoord);
            this.ComputeStats(HandRight_arr, HandRightStats_arr, Ycoord);
            this.ComputeStats(HandLeft_arr, HandLeftStats_arr, Ycoord);
            //knees
            this.ComputeStats(KneeRight_arr, HandRightStats_arr, Xcoord);
            this.ComputeStats(KneeLeft_arr, HandLeftStats_arr, Xcoord);
            this.ComputeStats(KneeRight_arr, HandRightStats_arr, Ycoord);
            this.ComputeStats(KneeLeft_arr, HandLeftStats_arr, Ycoord);
            
            //get x and y variance of joints
            double HRvarX = HandRightStats_arr[VARcoord,Xcoord];
            double HRvarY = HandRightStats_arr[VARcoord,Ycoord];
            double HLvarX = HandLeftStats_arr[VARcoord,Xcoord];
            double HLvarY = HandLeftStats_arr[VARcoord,Ycoord];
            double KRvarX = KneeRightStats_arr[VARcoord,Xcoord];
            double KRvarY = KneeRightStats_arr[VARcoord,Ycoord];
            double KLvarX = KneeLeftStats_arr[VARcoord,Xcoord];
            double KLvarY = KneeLeftStats_arr[VARcoord,Ycoord];
            bool isJumpingJackHands = (HRvarX > JJ_HANDTHRESH) && (HRvarY > JJ_HANDTHRESH) &&
                                (HLvarX > JJ_HANDTHRESH) && (HLvarY > JJ_HANDTHRESH);    
            bool isJumpingJackKnees = (KRvarX > JJ_KNEETHRESH) && (KRvarY > JJ_KNEETHRESH) &&
                                (KLvarX > JJ_KNEETHRESH) && (KLvarY > JJ_KNEETHRESH);                             

            System.Console.WriteLine(
                "JJ" + "\t" + 
                this.HandRightStats_arr[SUMcoord,Ycoord] + "\t" + 
                this.HandRightStats_arr[SOScoord,Ycoord] + "\t" + 
                this.HandRightStats_arr[VARcoord,Ycoord] + "\t" +
                isJumpingJackHands + "\t" + isJumpingJackKnees + "\t");

            return isJumpingJackHands && isJumpingJackKnees;
        }

        // returns true if arms are straight and moving vertically and not moving horizontally
        private bool isArmCircle(Skeleton skeleton, int index)
        {
            bool armLeftStraight = isArmStraight(skeleton, JointType.WristLeft, JointType.ElbowLeft, JointType.ShoulderLeft);
            bool armRightStraight = isArmStraight(skeleton, JointType.WristRight, JointType.ElbowRight, JointType.ShoulderRight);

            this.ComputeStats(HandRight_arr, HandRightStats_arr, Xcoord);
            this.ComputeStats(HandLeft_arr, HandLeftStats_arr, Xcoord);
            this.ComputeStats(HandRight_arr, HandRightStats_arr, Ycoord);
            this.ComputeStats(HandLeft_arr, HandLeftStats_arr, Ycoord);

            double HRvarX = HandRightStats_arr[VARcoord, Xcoord];
            double HRvarY = HandRightStats_arr[VARcoord, Ycoord];
            double HLvarX = HandLeftStats_arr[VARcoord, Xcoord];
            double HLvarY = HandLeftStats_arr[VARcoord, Ycoord];

            bool straightArms = armRightStraight && armLeftStraight;
            bool vertMvt = (HRvarY > AC_YHANDTHRESH) && (HLvarY > AC_YHANDTHRESH);
            bool horzMvt = (HRvarX < AC_XHANDTHRESH) && (HLvarX < AC_XHANDTHRESH);
            bool isArmCircle = straightArms && vertMvt && horzMvt;                                

            System.Console.WriteLine("AC" + "\t" +
                HRvarY + "\t" + HLvarY + "\t" + HRvarX + "\t" + HLvarX + "\t" +
                straightArms + "\t" + vertMvt + "\t" + horzMvt + "\t" + isArmCircle);

            return isArmCircle;
        }

        /// returns true if right hand is moving and left hand is near hip
        private bool isDisco(Skeleton skeleton)
        {
            //update sum, sum of squares, and variance for right hand
            this.ComputeStats(HandRight_arr, HandRightStats_arr, Xcoord);
            this.ComputeStats(HandRight_arr, HandRightStats_arr, Ycoord);

            //get x and y variance of joints
            double HRvarX = HandRightStats_arr[VARcoord,Xcoord];
            double HRvarY = HandRightStats_arr[VARcoord,Ycoord];

            bool rh_moving = (HRvarX > DSC_HRTHRESH) && (HRvarY > DSC_HRTHRESH);
            double HandLeftToHipDist = this.GetJointDistance(skeleton, JointType.HandLeft, JointType.HipLeft);
            bool HandLeftNearHip = HandLeftToHipDist < DSC_HLTHRESH;
            bool isDisco = rh_moving && HandLeftNearHip;

            System.Console.WriteLine("DISCO" + "\t" + 
                HandLeftToHipDist + "\t" +
                HRvarY + "\t" + HRvarX + "\t" +
                rh_moving + "\t" + HandLeftNearHip + "\t");

            return isDisco;
        }

        /// returns true if hands are moving in XZ plane and not in Y plane
        private bool isCabbagePatch(Skeleton skelton)
        {
            //update sum, sum of squares, and variance
            this.ComputeStats(HandRight_arr, HandRightStats_arr, Xcoord);
            this.ComputeStats(HandLeft_arr, HandLeftStats_arr, Xcoord);
            this.ComputeStats(HandRight_arr, HandRightStats_arr, Ycoord);
            this.ComputeStats(HandLeft_arr, HandLeftStats_arr, Ycoord);
            this.ComputeStats(HandRight_arr, HandRightStats_arr, Zcoord);
            this.ComputeStats(HandLeft_arr, HandLeftStats_arr, Zcoord);
            
            //get x and z variance of hands
            double HRvarX = HandRightStats_arr[VARcoord,Xcoord];
            double HRvarY = HandRightStats_arr[VARcoord,Ycoord];
            double HRvarZ = HandRightStats_arr[VARcoord,Zcoord];
            double HLvarX = HandLeftStats_arr[VARcoord,Xcoord];
            double HLvarY = HandLeftStats_arr[VARcoord,Ycoord];
            double HLvarZ = HandLeftStats_arr[VARcoord,Zcoord];

            bool Xaxis = (HRvarX > CP_XHANDTHRESH) && (HLvarX > CP_XHANDTHRESH);
            bool Yaxis = (HRvarY < CP_YHANDTHRESH) && (HLvarY < CP_YHANDTHRESH);
            bool Zaxis = (HRvarZ > CP_ZHANDTHRESH) && (HLvarZ > CP_ZHANDTHRESH);    
            bool isCabbage = Xaxis && Yaxis && Zaxis;                             

            System.Console.WriteLine(
                "CABBAGE\tX " + Xaxis + "\t" + HRvarX + "\t" + HLvarX + "\t" + 
                "Y " + Yaxis + "\t" + HRvarY+ "\t" + HLvarY + "\t" + 
                "Z " + Zaxis + "\t" + HRvarZ+ "\t" + HLvarZ + "\t" + 
                isCabbage);

            return isCabbage;
        }

        //log coordinates of all joints
        private void TrackJoints(Skeleton skeleton, int index)
        {
            //track hands
            LogJointPosition(skeleton, JointType.HandRight, HandRight_arr, HandRightStats_arr, index, Xcoord);
            LogJointPosition(skeleton, JointType.HandRight, HandRight_arr, HandRightStats_arr, index, Ycoord);
            LogJointPosition(skeleton, JointType.HandLeft, HandLeft_arr, HandLeftStats_arr, index, Xcoord);
            LogJointPosition(skeleton, JointType.HandLeft, HandLeft_arr, HandLeftStats_arr, index, Ycoord);
            //track knees
            LogJointPosition(skeleton, JointType.KneeRight, KneeRight_arr, KneeRightStats_arr, index, Xcoord);
            LogJointPosition(skeleton, JointType.KneeRight, KneeRight_arr, KneeRightStats_arr, index, Ycoord);
            LogJointPosition(skeleton, JointType.KneeLeft, KneeLeft_arr, KneeLeftStats_arr, index, Xcoord);
            LogJointPosition(skeleton, JointType.KneeLeft, KneeLeft_arr, KneeLeftStats_arr, index, Ycoord);
            
            if (this.danceState == (int) danceStateOptions.Cabbage)
            {   // only log the z coordinate if the dance move is cabbage patch
                LogJointPosition(skeleton, JointType.HandRight, HandRight_arr, HandRightStats_arr, index, Zcoord);
                LogJointPosition(skeleton, JointType.HandLeft, HandLeft_arr, HandLeftStats_arr, index, Zcoord);
            }
        }

        private void validate(Skeleton skeleton)
        {
            bool isCorrect = false;
            // System.Console.WriteLine(this.danceState);

            if (DateTime.Compare(DateTime.Now, GracePeriodEndTime) < 0)
            {   //current time is before the grace period end time, default to true
                isCorrect = true;
            }

            else
            {
                switch (this.danceState)
                {   //current time is after the grace period end time
                    case (int)danceStateOptions.JumpingJack:
                        //System.Console.WriteLine("do jumping jack!");
                        isCorrect = isJumpingJack();
                        break;
                    case (int)danceStateOptions.ArmCircle:
                        // System.Console.WriteLine("do arm circles!");
                        isCorrect = isArmCircle(skeleton, index);
                        break;
                    case (int)danceStateOptions.Disco:
                        // System.Console.WriteLine("do disco!");
                        isCorrect = isDisco(skeleton);
                        break;
                    case (int)danceStateOptions.Cabbage:
                        // System.Console.WriteLine("do cabbage patch!");
                        isCorrect = isCabbagePatch(skeleton);
                        break;
                    default:
                        break;
                }
            }

            if(isCorrect){validationCorrectUI();}
            else{validationIncorrectUI();lives -= 1; } 
        }

        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);

                    if (skeletons.Length != 0)
                    {
                        foreach (Skeleton skeleton in skeletons)
                        {
                            if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                            {
                                this.TrackJoints(skeleton, index);
                                if (index == ARRLEN - 1)
                                {   // validate move when we have updated a complete row
                                    this.validate(skeleton);
                                }
                                index++;
                                index = index % ARRLEN;
                                //System.Console.WriteLine(index);
                            }
                        }
                    }     
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// Draws a skeleton's bones and joints
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);
 
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;                    
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;                    
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// Draws a bone line between two joints
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        void onClick1(object sender, RoutedEventArgs e)
        {
            //hide button
            uniformGrid2.Visibility = Visibility.Visible;
            uniformGrid.Visibility = Visibility.Hidden;

            //start dance
            this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/jumpingjack.png", UriKind.Relative));
            this.danceState = (int)danceStateOptions.JumpingJack;
            DanceMove.Source = this.danceImage;
            player.Play();
            // Create a timer with a two second interval.
            aTimer = new System.Timers.Timer(200); 
            // Hook up the Elapsed event for the timer. 
            aTimer.Elapsed += OnTimedEvent;
            aTimer.AutoReset = false;
            //aTimer.Enabled = true;
            aTimer.Start();

        }

        private void OnTimedEvent(Object source, ElapsedEventArgs e)
        {
            //Console.WriteLine("The Elapsed event was raised at {0:HH:mm:ss.fff}", e.SignalTime);
            Dispatcher.Invoke((Action)delegate() { 
                if(this.danceState == (int)danceStateOptions.JumpingJack){
                    this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/jumpingjack.png",UriKind.Relative));
                    this.danceState = (int)danceStateOptions.ArmCircle;
                } else if(this.danceState == (int)danceStateOptions.ArmCircle){
                    this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/armcircle.png",UriKind.Relative));
                    this.danceState = (int)danceStateOptions.Disco;
                } else if(this.danceState == (int)danceStateOptions.Disco){
                    this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/disco.png",UriKind.Relative));
                    //this.danceState = (int)danceStateOptions.JumpingJack;
                    this.danceState = (int)danceStateOptions.Cabbage;
                } else if(this.danceState == (int)danceStateOptions.Cabbage){
                    this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/cabbage.png",UriKind.Relative));
                    this.danceState = (int)danceStateOptions.Balloons;
                } else { //if(this.danceState == (int)danceStateOptions.Balloons){
                    this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/balloons.jpg",UriKind.Relative));
                    //this.danceState = (int)danceStateOptions.JumpingJack;
                    this.danceState = (int)danceStateOptions.Balloons;
                    aTimer.Stop();
                    endgame();
                }

                DanceMove.Source = this.danceImage;
                //set grace period
                GracePeriodEndTime = DateTime.Now.AddSeconds(2.0);

                aTimer = new System.Timers.Timer(timePerDanceMove); 
                // Hook up the Elapsed event for the timer. 
                aTimer.Elapsed += OnTimedEvent;
                aTimer.AutoReset = false;
                //aTimer.Enabled = true;
                aTimer.Start();
            });
        }

        void onClick2(object sender, RoutedEventArgs e){
            uniformGrid2.Visibility = Visibility.Visible;
            uniformGrid.Visibility = Visibility.Hidden;
        }

        /// a onclick function to test how to play music
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        void playSong(object sender, RoutedEventArgs e){
            //var pathURI = new Uri(@"Song/chickenDance.wav",UriKind.Relative);
            //var path = pathURI.LocalPath;
            var path = @"C:\Users\amyli\just-heat-it\SkeletonBasics-WPF\Song\blurredlines.wav";
            SoundPlayer player = new SoundPlayer(path);
            player.Load();
            player.Play();
        }    
        
        void arduinoSetup(){
            string[] ports = SerialPort.GetPortNames();
            //if ports is empty. print error
            if(ports.Length == 0) {System.Diagnostics.Debug.WriteLine("[arduinoSetup] ERROR: no ports found");}

            foreach(string portName in ports) {                
                port = new SerialPort(portName, 9600, Parity.None, 8, StopBits.One);
                port.Open();

                serialDebugVar = (int)serialMessageOptions.CorrectMove; //for debug

                /*
                note: run arduino first. 
                open and run visual studio project second. 
                should not be able to open arduino monitor. 
                */
            }
        }

        void onClick3(object sender, RoutedEventArgs e){
            //TODO: run this with arduino and check that it still works. 
            if(serialDebugVar == (int)serialMessageOptions.CorrectMove) {
                if(ARDUINO_CONNECTED) port.Write(new byte[] {(byte)(int)serialMessageOptions.IncorrectMove}, 0, 1);
                serialDebugVar = (int)serialMessageOptions.IncorrectMove;
            } else {
                if(ARDUINO_CONNECTED) port.Write(new byte[] {(byte)(int)serialMessageOptions.CorrectMove}, 0, 1);
                serialDebugVar = (int)serialMessageOptions.CorrectMove;
            }
        }

        void validationCorrectUI(){
            //change to checkmark picture
            this.checkOrRed.Source = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images\check.png",UriKind.Relative));
                    
            //signal to arduino
            if(ARDUINO_CONNECTED) port.Write(new byte[] {(byte)(int)serialMessageOptions.CorrectMove}, 0, 1);
        }

        void validationIncorrectUI(){
            //change to checkmark picture
            this.checkOrRed.Source = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images\red.jpeg",UriKind.Relative));
                    
            //signal to arduino
            if(ARDUINO_CONNECTED) port.Write(new byte[] {(byte)(int)serialMessageOptions.IncorrectMove}, 0, 1);
        }

        void newGame(object sender, RoutedEventArgs e){
            //returns to game start mode
            uniformGrid2.Visibility = Visibility.Hidden;
            uniformGrid.Visibility = Visibility.Visible;
            aTimer.Stop();
            player.Stop();
        }

        void gameGracePeriod(object sender, RoutedEventArgs e){
            //hide button
            uniformGrid.Visibility = Visibility.Hidden;
            GameGraceText.Visibility = Visibility.Visible;
/*
            while(countdown > 0){
                aTimer = new System.Timers.Timer(1000);
                // Hook up the Elapsed event for the timer. 
                aTimer.Elapsed += decrementCountdown;
                aTimer.AutoReset = false;
                aTimer.Enabled = true;

                GameGraceCountdown.Text = countdown.toString();
            }
            */

            Timer bTimer = new System.Timers.Timer(2000);
            // Hook up the Elapsed event for the timer. 
            bTimer.Elapsed += showGo;
            bTimer.AutoReset = false;
            bTimer.Enabled = true;

            aTimer = new System.Timers.Timer(3000);
            // Hook up the Elapsed event for the timer. 
            aTimer.Elapsed += startDance;
            aTimer.AutoReset = false;
            aTimer.Enabled = true;
            

            //start dance
            //startDance();
        }

        void decrementCountdown(Object source, ElapsedEventArgs e){
            countdown--;
        }

        void showGo(Object source, ElapsedEventArgs e){
            Dispatcher.Invoke((Action)delegate() { 
                GameGraceCountdown.Visibility = Visibility.Visible;
            });
        }

        void startDance(Object source, ElapsedEventArgs e){
            Dispatcher.Invoke((Action)delegate() { 
                //hide grace period text
                GameGraceText.Visibility = Visibility.Hidden;
                uniformGrid2.Visibility = Visibility.Visible;

                //start dance
                this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/jumpingjack.png", UriKind.Relative));
                this.danceState = (int)danceStateOptions.JumpingJack;
                DanceMove.Source = this.danceImage;
            });

                player.Play();
                // Create a timer with a two second interval.
                aTimer = new System.Timers.Timer(200); 
                // Hook up the Elapsed event for the timer. 
                aTimer.Elapsed += OnTimedEvent;
                aTimer.AutoReset = false;
                //aTimer.Enabled = true;
                aTimer.Start();
        }

        //turns off microwave
        void endgame(){
            //if(ARDUINO_CONNECTED) port.Write(new byte[] {(byte)(int)serialMessageOptions.CorrectMove}, 0, 1);
            player.Stop();
        }
    }
}