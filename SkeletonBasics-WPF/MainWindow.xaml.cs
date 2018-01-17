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

        /// dance move prompt
        System.Windows.Media.Imaging.BitmapImage danceImage;
        private int danceState;
        enum danceStateOptions {ChickenMove1, ChickenMove2, ChickenMove3, ChickenMove4, JumpingJack, Disco};
        private static System.Timers.Timer aTimer;

        //communication with arduino SERIAL 
        SerialPort port;
        int serialDebugVar; //for serial demo/debug
        enum serialMessageOptions {doNotUse, CorrectMove, IncorrectMove};

        /// global index counter 
        private int index = 0;
        //private bool endOfRow = false;
        private const int ARRLEN = 50;
        private const int Xcoord = 0;
        private const int Ycoord = 1;
        private const int SUMcoord = 0;
        private const int SOScoord = 1;
        private const int VARcoord = 2;
        /// 2D array for (x,y) coordinates; 2 rows, 10 cols
        private double[,] HandRight_arr = new double[2, ARRLEN];
        private double[,] HandLeft_arr = new double[2, ARRLEN];
        private double[,] KneeRight_arr = new double[2, ARRLEN];
        private double[,] KneeLeft_arr = new double[2, ARRLEN];
        /// (SUM/SOS/VAR , X/Y)
        private double[,] HandRightStats_arr = new double[3,2];
        private double[,] HandLeftStats_arr = new double[3,2];
        private double[,] KneeRightStats_arr = new double[3,2];
        private double[,] KneeLeftStats_arr = new double[3,2];

        //variance thresholds
        private const double JJ_HANDTHRESH = .0001;  //variance
        private const double JJ_KNEETHRESH = .0000001;  //variance
        private const double DSC_HRTHRESH = .0001;   //variance
        private const double DSC_HLTHRESH = .5;     //dist bw left hand and lef hip
        private const double STRAIGHT_DELTA = .2;


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
            
            talkToArduino();

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
            port.Close();
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

        /// Return X position
        private double GetJointX(Skeleton skeleton, JointType jointType)
        {
            Joint j = skeleton.Joints[jointType];
            // If we can't find the joint, exit
            if (j.TrackingState == JointTrackingState.NotTracked) return 0;
            
            return j.Position.X;
        }

        /// Return Y position
        private double GetJointY(Skeleton skeleton, JointType jointType)
        {
            Joint j = skeleton.Joints[jointType];
            // If we can't find the joint, exit
            if (j.TrackingState == JointTrackingState.NotTracked) return 0;
            
            return j.Position.Y;
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

        /// Update X coordinate, sum, sum of squares, and variance of a joint
        private void LogJointPositionX(Skeleton skeleton, JointType jointType, 
                                        double[,] arr, double[,] arrConst, int index)
        {
            Joint j = skeleton.Joints[jointType];
            // If we can't find the joint, exit
            if (j.TrackingState == JointTrackingState.NotTracked) return;
            //update array of coordinates
            arr[Xcoord,index] = this.GetJointX(skeleton, jointType);
        }

        /// Update Y coordinate of a joint
        private void LogJointPositionY(Skeleton skeleton, JointType jointType, 
                                        double[,] arr, double[,] arrConst, int index)
        {
            Joint j = skeleton.Joints[jointType];
            // If we can't find the joint, exit
            if (j.TrackingState == JointTrackingState.NotTracked) return;
            //update array of coordinates
            arr[Ycoord,index] = this.GetJointY(skeleton, jointType);           
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
            double wristX = GetJointX(skeleton, wrist);
            double wristY = GetJointY(skeleton, wrist);
            double elbowX = GetJointX(skeleton, elbow);
            double elbowY = GetJointY(skeleton, elbow);
            double shoulderX = GetJointX(skeleton, shoulder);
            double shoulderY = GetJointY(skeleton, shoulder);

            double slope1 = GetSlope(wristX, elbowX, wristY, elbowY);
            double slope2 = GetSlope(elbowX, shoulderX, elbowY, shoulderY);

            return System.Math.Abs(slope1 - slope2) < STRAIGHT_DELTA;
        }


        /// Return true if elbows are near torso
        /// (distance between elbows is within 0.3m of distance between shoulders)
        private bool IsValidChknMove1(Skeleton skeleton)
        {
            Joint ShoulderLeft = skeleton.Joints[JointType.ShoulderLeft];
            Joint ShoulderRight = skeleton.Joints[JointType.ShoulderRight];
            Joint ElbowLeft = skeleton.Joints[JointType.ElbowLeft];
            Joint ElbowRight = skeleton.Joints[JointType.ElbowRight];

            double shoulderWidth = this.GetJointDistance(skeleton, JointType.ShoulderLeft, JointType.ShoulderRight);
            double elbowDistance = this.GetJointDistance(skeleton, JointType.ElbowLeft, JointType.ElbowRight);

            // If we can't find either of these joints, exit
            if (shoulderWidth < 0 || elbowDistance < 0) return false;

            //if elbows are close enough to torso
            if (shoulderWidth + 0.3 > elbowDistance) 
            {
                ///System.Console.WriteLine(shoulderWidth + '\t' + elbowDistance);
                return true;
            }
            else return false;

        }

        /// Return true if hands are near shoulders
        /// (hands are within 0.2m of shoulders)
        private bool IsValidChknMove2(Skeleton skeleton)
        {
            Joint ShoulderLeft = skeleton.Joints[JointType.ShoulderLeft];
            Joint ShoulderRight = skeleton.Joints[JointType.ShoulderRight];
            Joint HandLeft = skeleton.Joints[JointType.HandLeft];
            Joint HandRight = skeleton.Joints[JointType.HandRight];

            double Left = this.GetJointDistance(skeleton, JointType.ShoulderLeft, JointType.HandLeft);
            double Right = this.GetJointDistance(skeleton, JointType.ShoulderRight, JointType.HandRight);

            // If we can't find either of these joints, exit
            if (Left < 0 || Right < 0) return false;

            //if elbows are close enough to torso
            if (Left < 0.2 && Right < 0.2) 
            {
                System.Console.WriteLine(Left + '\t' + Right);
                return true;
            }
            else return false;

        }

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

        /// right hand is moving and left hand is near hip
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

            System.Console.Write(
                "DISCO" + "\t" + 
                this.HandRightStats_arr[VARcoord,Ycoord] + "\t" +
                HandLeftToHipDist + "\t" +
                rh_moving + "\t" + HandLeftNearHip + "\t");

            return isDisco;
        }

        //private bool isArmCircle(Skeleton skeleton, int index)

        //log coordinates of all joints
        private void TrackJoints(Skeleton skeleton, int index)
        {
            //track hands
            LogJointPositionX(skeleton, JointType.HandRight, HandRight_arr, HandRightStats_arr, index);
            LogJointPositionY(skeleton, JointType.HandRight, HandRight_arr, HandRightStats_arr, index);
            LogJointPositionX(skeleton, JointType.HandLeft, HandLeft_arr, HandLeftStats_arr, index);
            LogJointPositionY(skeleton, JointType.HandLeft, HandLeft_arr, HandLeftStats_arr, index);
            //track knees
            LogJointPositionX(skeleton, JointType.KneeRight, KneeRight_arr, KneeRightStats_arr, index);
            LogJointPositionY(skeleton, JointType.KneeRight, KneeRight_arr, KneeRightStats_arr, index);
            LogJointPositionX(skeleton, JointType.KneeLeft, KneeLeft_arr, KneeLeftStats_arr, index);
            LogJointPositionY(skeleton, JointType.KneeLeft, KneeLeft_arr, KneeLeftStats_arr, index);
        }

        private void validate(Skeleton skeleton)
        {
            bool isCorrect = false;
            System.Console.WriteLine(this.danceState);

            switch(this.danceState){
                case (int)danceStateOptions.JumpingJack:
                    System.Console.WriteLine("is jumping jack!");
                    isCorrect = isJumpingJack();
                    break;
                case (int)danceStateOptions.Disco:
                    System.Console.WriteLine("is disco!");
                    isCorrect = isDisco(skeleton);
                    break;
                default: 
                    break;
            }
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
                                if (index == ARRLEN-1) this.validate(skeleton);
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

        /// Handles the checking or unchecking of the seated mode combo box
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                /*if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                } */
                this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
            }
        }

        void onClick1(object sender, RoutedEventArgs e)
        {
            //hide button
            uniformGrid2.Visibility = Visibility.Visible;
            uniformGrid.Visibility = Visibility.Hidden;

            //start dance
            this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/dance1.png", UriKind.Relative));
            this.danceState = (int)danceStateOptions.JumpingJack;
            DanceMove.Source = this.danceImage;
            // Create a timer with a two second interval.
            aTimer = new System.Timers.Timer(3000);
            // Hook up the Elapsed event for the timer. 
            aTimer.Elapsed += OnTimedEvent;
            aTimer.AutoReset = true;
            aTimer.Enabled = true;
            //aTimer.Start();

        }

        private void OnTimedEvent(Object source, ElapsedEventArgs e)
        {
            //Console.WriteLine("The Elapsed event was raised at {0:HH:mm:ss.fff}", e.SignalTime);
            Dispatcher.Invoke((Action)delegate() { 
                /*
                if(this.danceState == (int)danceStateOptions.ChickenMove1){
                    this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/dance2.png",UriKind.Relative));
                    this.danceState = (int)danceStateOptions.ChickenMove2;
                } else if(this.danceState == (int)danceStateOptions.ChickenMove2){
                    this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/dance3.png",UriKind.Relative));
                    this.danceState = (int)danceStateOptions.ChickenMove3;
                } else if(this.danceState == (int)danceStateOptions.ChickenMove3){
                    this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/dance4.png",UriKind.Relative));
                    this.danceState = (int)danceStateOptions.ChickenMove4;
                } else { //if(this.danceState == danceStateOptions.ChickenMove4){
                    this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/dance1.png",UriKind.Relative));
                    this.danceState = (int)danceStateOptions.ChickenMove1;
                }
                */

                //leave as chicken picture until get jumping jack picture
                this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/dance2.png",UriKind.Relative));
                this.danceState = (int)danceStateOptions.Disco;
                DanceMove.Source = this.danceImage;
            });

            //change dance move 1 to 2
            //this.danceImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images/moves/dance2.png",UriKind.Relative));
            //DanceMove.Source = this.danceImage;

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
            var path = @"C:\Users\amyli\just-heat-it\SkeletonBasics-WPF\Song\chickenDance.wav";
            SoundPlayer player = new SoundPlayer(path);
            player.Load();
            player.Play();
        }    
        
        void talkToArduino(){
            //SerialPort serialPort1 = new SerialPort();
            //serialPort1.Open();

            // Get a list of serial port names.
            string[] ports = SerialPort.GetPortNames();

             System.Diagnostics.Debug.WriteLine("The following serial ports were found:");

            // Display each port name to the console.
            foreach(string portName in ports)
            {
                 System.Diagnostics.Debug.WriteLine(portName);

                // Instantiate the communications
              // port with some basic settings
              port = new SerialPort(portName, 9600, Parity.None, 8, StopBits.One);
  
              // Open the port for communications
              port.Open();
  
              // Write a string
              //port.Write("Hello World");
  
              // Write a set of bytes
              //port.Write(new byte[] {0x01}, 0, 1);
  
              // Close the port
              //port.Close();
              //move ^ to window close

            serialDebugVar = (int)serialMessageOptions.CorrectMove; //for debug

            /*
            note: run arduino first. 
            open and run visual studio project second. 
            should not be able to open arduino monitor. 
            */
            }

            
            //Console.ReadLine();
        }

        void onClick3(object sender, RoutedEventArgs e){
            //TODO: run this with arduino and check that it still works. 
            if(serialDebugVar == (int)serialMessageOptions.CorrectMove) {
                port.Write(new byte[] {(byte)(int)serialMessageOptions.CorrectMove}, 0, 1);
                serialDebugVar = (int)serialMessageOptions.IncorrectMove;
            } else {
                port.Write(new byte[] {(byte)(int)serialMessageOptions.CorrectMove}, 0, 1);
                serialDebugVar = (int)serialMessageOptions.CorrectMove;
            }
        }

        void validationCorrectUI(){
            //change to checkmark picture
            this.checkOrRed = new System.Windows.Media.Imaging.BitmapImage(new Uri(@"Images\check.png",UriKind.Relative));
                    
            //TODO: signal to arduino
        }
    }
}