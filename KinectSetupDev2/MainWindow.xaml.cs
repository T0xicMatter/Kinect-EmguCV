using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Drawing;
using Microsoft.Kinect;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.UI;
using System.IO;


namespace KinectSetupDev
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        bool closing = false;
        const int skeletonCount = 6;
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser1_KinectSensorChanged);
        }

        void kinectSensorChooser1_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            KinectSensor oldSensor = (KinectSensor)e.OldValue;
            StopKinect(oldSensor);

            KinectSensor newSensor = (KinectSensor)e.NewValue;

            newSensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            newSensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            newSensor.SkeletonStream.Enable();
            
            newSensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(_sensor_AllFramesReady);
            try
            {
                newSensor.Start();
            }
            catch (System.IO.IOException)
            {
                kinectSensorChooser1.AppConflictOccurred();
                
            }
        }


        void _sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            if (closing)
            {
                return;
            }

            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame == null)
                {
                    return;
                }

                byte[] pixels = new byte[colorFrame.PixelDataLength];
                colorFrame.CopyPixelDataTo(pixels);

                int stride = colorFrame.Width * 4;
                BitmapSource bsO= BitmapSource.Create(colorFrame.Width, colorFrame.Height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
                BitmapSource bsP;
                //// ImgOriginal & ImgProcessed are WPF Image Controls
                //// ImgO & ImgP are EmguCV Image<>
                
                //  1. Convert BitmapSource To Image<>
                Image<Bgr, Byte> ImgO = BitmapSourceToImage(bsO);
                Image<Gray, Byte> ImgP;
                // 2. Do image processing on Image<> imgO

                ImgP = ImgO.InRange(new Bgr(0, 0, 175),                      //max filter value
                                             new Bgr(100, 100, 256));

                ImgP = ImgP.SmoothGaussian(9);                              //SmoothGaussian is called with only one param ie. the x & y size of the filter window
                CircleF[] circles = ImgP.HoughCircles(new Gray(100),                //Canny threshold
                                                              new Gray(50),                 //accumulator threshold
                                                              2,                            //size of img / this param = accumulator resolution
                                                              imgProcessed.Height / 4,      //min distance in pixels between centres of the detected circles
                                                              10,                           //min radius of detected circles
                                                              400)[0];
                foreach (CircleF circle in circles)
                {
                    if (txtXYRadius.Text != "")
                    {
                        txtXYRadius.AppendText(Environment.NewLine);                        //if we are not on first line, insert a new line character
                    }

                    txtXYRadius.AppendText("ball position = x:" + circle.Center.X.ToString().PadLeft(4) +
                        ", y:" + circle.Center.X.ToString().PadLeft(4) +
                        ", radius = " + circle.Radius.ToString("###.000").PadLeft(7));

                    txtXYRadius.ScrollToEnd();

                    //draw a small green circle at the center of the detected object
                    //to do this, we will call the OpenCV 1.x function. 
                    //this is necessary b/c we are drawing circle of radius 3, even thoug the size
                    //of the detected circle will be much bigger
                    //the CVInvoke  object can be used to make OpenCV 1.x function calls

                    CvInvoke.cvCircle(ImgO,
                                       new System.Drawing.Point((int)circle.Center.X, (int)circle.Center.Y),
                                       3,                                     //radius of circle          
                                       new MCvScalar(0, 255, 0),
                                       -1,                                    //thickness of circle in pixel, -1 indicates to fill the circle          
                                       LINE_TYPE.CV_AA,                       //use AA to smooth the circles
                                       0);                                    //no shift
                    //draw a red circle around the detected object
                    ImgO.Draw(circle, new Bgr(System.Drawing.Color.Red), 3);
                }





                // 3. Convert Image<> To Bitmap and then Bitmap to BitmapSource
                bsO = BitmapToBitmapSource(ImgO.Bitmap);
                bsP = BitmapToBitmapSource(ImgP.Bitmap);
                //4. Give BitmapSoure to WPF Image
                imgOriginal.Source = bsO;
                imgProcessed.Source = bsP;
            }
            
              ////FOR DEPTH:
              using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
              {
                  if (depthFrame == null)
                  {
                      return;
                  }

                  byte[] pixels = GenerateColoredBytes(depthFrame);    
                  int stride = depthFrame.Width * 4;
                  DImage.Source = BitmapSource.Create(depthFrame.Width, depthFrame.Height, 192, 192, PixelFormats.Bgr32, null, pixels, stride);
              }
              

            Skeleton first = GetFirstSkeleton(e);

            if (first == null)
            {
                return;
            }

            GetCameraPoint(first, e);            
            
        }

        //BitmapSource To Image<>
        Image<Bgr,Byte> BitmapSourceToImage(BitmapSource bitmapsource)
        {
            using (MemoryStream outStream = new MemoryStream())
            {
                //MemoryStream outStream = new MemoryStream();
                BitmapEncoder enc = new BmpBitmapEncoder();
                enc.Frames.Add(BitmapFrame.Create(bitmapsource));
                enc.Save(outStream);
                System.Drawing.Bitmap bitmap = new System.Drawing.Bitmap(outStream);

                Image<Bgr, Byte> sampleImg = new Image<Bgr, Byte>(bitmap);
                return sampleImg;
            }
        }
            
        //Bitmap To BitmapSource
        [DllImport("gdi32.dll")]
        private static extern bool DeleteObject(IntPtr hObject);
        
        public static BitmapSource BitmapToBitmapSource(System.Drawing.Bitmap bitmap)
        {
            if (bitmap == null)
                throw new ArgumentNullException("bitmap");


            lock (bitmap)
            {
                IntPtr hBitmap = bitmap.GetHbitmap();

                try
                {
                    return System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(hBitmap, IntPtr.Zero, Int32Rect.Empty, BitmapSizeOptions.FromEmptyOptions());
                }
                finally
                {
                    DeleteObject(hBitmap);
                }
            }
        }

           

                //For DEPTH:

        private byte[] GenerateColoredBytes(DepthImageFrame depthFrame)
        {
            short[] rawDepthData = new short[depthFrame.PixelDataLength];
            depthFrame.CopyPixelDataTo(rawDepthData);

            Byte[] pixels = new byte[depthFrame.Height * depthFrame.Width * 4];

            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;

            for (int depthIndex = 0, colorIndex = 0;
                depthIndex < rawDepthData.Length && colorIndex < pixels.Length;
                depthIndex++, colorIndex += 4)
            {
                int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;
                int depth = rawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;
              
                if (depth <= 900)
                {
                    pixels[colorIndex + BlueIndex] = 255;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 0;
                }
                else if (depth > 2000)
                {
                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 255;
                }

                byte intensity = CalculateIntensityFromDepth(depth);
                pixels[colorIndex + BlueIndex] = intensity;
                pixels[colorIndex + GreenIndex] = intensity;
                pixels[colorIndex + RedIndex] = intensity;

                if (player > 0)
                {
                    pixels[colorIndex + BlueIndex] = 33;
                    pixels[colorIndex + GreenIndex] = 33;
                    pixels[colorIndex + RedIndex] = 33;
                }
            }
            /*==>*/
            
            return pixels;
        }

        public static byte CalculateIntensityFromDepth(int distance)
        {
            return (byte)(255 - (255 * Math.Max(distance - 400, 0) / (8000)));
        }
        //DEPTH_END
            

          void GetCameraPoint(Skeleton first, AllFramesReadyEventArgs e)
          {
              using (DepthImageFrame depth = e.OpenDepthImageFrame())
              {
                  if (depth == null || kinectSensorChooser1.Kinect == null)
                  {
                      return;
                  }
                  CoordinateMapper cm = new CoordinateMapper(kinectSensorChooser1.Kinect);
                  DepthImagePoint headDepthPoint = cm.MapSkeletonPointToDepthPoint(first.Joints[JointType.Head].Position,DepthImageFormat.Resolution640x480Fps30);
                  DepthImagePoint leftDepthPoint = cm.MapSkeletonPointToDepthPoint(first.Joints[JointType.HandLeft].Position, DepthImageFormat.Resolution640x480Fps30);
                  DepthImagePoint rightDepthPoint = cm.MapSkeletonPointToDepthPoint(first.Joints[JointType.HandRight].Position, DepthImageFormat.Resolution640x480Fps30);
                
                  ColorImagePoint headColorPoint =  cm.MapDepthPointToColorPoint(DepthImageFormat.Resolution640x480Fps30, headDepthPoint, ColorImageFormat.RgbResolution640x480Fps30);
                  ColorImagePoint leftColorPoint = cm.MapDepthPointToColorPoint(DepthImageFormat.Resolution640x480Fps30, leftDepthPoint, ColorImageFormat.RgbResolution640x480Fps30);
                  ColorImagePoint rightColorPoint = cm.MapDepthPointToColorPoint(DepthImageFormat.Resolution640x480Fps30, rightDepthPoint, ColorImageFormat.RgbResolution640x480Fps30);
               
                //=>  CameraPosition(headImage, headColorPoint);
                //=>  CameraPosition(leftEllipse, leftColorPoint);
                //=>  CameraPosition(rightEllipse, rightColorPoint);
              }
          }
        
          private void CameraPosition(FrameworkElement element, ColorImagePoint point)
          {            
              Canvas.SetLeft(element, point.X - element.Width / 2);
              Canvas.SetTop(element, point.Y - element.Height / 2);
          }


          Skeleton GetFirstSkeleton(AllFramesReadyEventArgs e)
          {
              using (SkeletonFrame skeletonFrameData = e.OpenSkeletonFrame())
              {
                  if (skeletonFrameData == null)
                  {
                      return null;
                  }

                  skeletonFrameData.CopySkeletonDataTo(allSkeletons);

                  Skeleton first1 = (from s in allSkeletons
                                           where s.TrackingState == SkeletonTrackingState.Tracked
                                           select s).FirstOrDefault();
                  return first1;
              }
          }
          

        void StopKinect(KinectSensor sensor)
        {
            if (sensor != null)
            {
                sensor.Stop();
                sensor.AudioSource.Stop();
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            StopKinect(kinectSensorChooser1.Kinect);
        }

    }
}
