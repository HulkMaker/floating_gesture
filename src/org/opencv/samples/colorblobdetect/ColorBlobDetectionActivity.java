package org.opencv.samples.colorblobdetect;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
//import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.samples.floating_gesture.R;
import org.opencv.video.KalmanFilter;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.util.Log;
//import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;
//import android.widget.Toast;

public class ColorBlobDetectionActivity extends Activity implements OnTouchListener, CvCameraViewListener2 {
    private static final String  TAG              = "OCVSample::Activity";
    private double beishu= 3;

    //private boolean              mIsColorSelected = false;
    //private Mat                  mRgba;
    //private Scalar               mBlobColorRgba;
    //private Scalar               mBlobColorHsv;
    //private ColorBlobDetector    mDetector;
    //private Mat                  mSpectrum;
    //private Size                 SPECTRUM_SIZE;
    //private Scalar               CONTOUR_COLOR;
    
    Mat src;
    FingerTipDetection mFingers;
    
    Bitmap btsci;
    Bitmap btpaper;
    Bitmap btstone;
    Bitmap btgun;
    
    Mat matsci;
    Mat matpaper;
    Mat matstone;
    Mat matgun;
    
    int category;
    boolean exist;
    /*Mat gray;
    Mat bin;
    Mat dst;
    List<Mat> lst;
    Mat binr;
    Mat binb;
    
    Mat binr_low;
    Mat binr_high;
    Mat binr_bin;
    Mat binb_low;
    Mat binb_high;
    Mat binb_bin;
    Mat hand_bin;  
    
    Mat dst_map;
    Core.MinMaxLocResult MMR;
    
    //Mat mDilatedMask;
	Mat mHierarchy;
	
	Mat MxCtr;
	Point dist_ind;
	Point pit;
	
	boolean push;
	boolean pushs;
	
	KalmanFilter FingerTrack;*/

    private CameraBridgeViewBase mOpenCvCameraView;

    private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    mOpenCvCameraView.setOnTouchListener(ColorBlobDetectionActivity.this);
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public ColorBlobDetectionActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.color_blob_detection_surface_view);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
        mOpenCvCameraView.setCvCameraViewListener(this);
    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        /*mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(200, 64);
        CONTOUR_COLOR = new Scalar(255,0,0,255);*/
    	int wd = width/(int)beishu;
    	int ht = height/(int)beishu;
    	src = new Mat(height,width,CvType.CV_8UC4);
    	mFingers = new FingerTipDetection();
    	mFingers.initialize();
    	/*gray = new Mat(ht,wd,CvType.CV_8U);
    	bin = new Mat(ht,wd,CvType.CV_8U);
    	
    	dst = new Mat(ht,wd,CvType.CV_8U);
    	binr = new Mat(ht,wd,CvType.CV_8U);
    	binb = new Mat(ht,wd,CvType.CV_8U);
    	lst = new ArrayList<Mat>();
    	hand_bin = new Mat(ht,wd,CvType.CV_8U);
    	binr_low = new Mat(ht,wd,CvType.CV_8U);
    	binr_high = new Mat(ht,wd,CvType.CV_8U);
    	binb_low = new Mat(ht,wd,CvType.CV_8U);
    	binb_high = new Mat(ht,wd,CvType.CV_8U);
    	binr_bin = new Mat(ht,wd,CvType.CV_8U);
    	binb_bin = new Mat(ht,wd,CvType.CV_8U);
    	dst_map = new Mat(ht,wd,CvType.CV_8U);
    	MMR = new Core.MinMaxLocResult();
    	//mDilatedMask = new Mat(height,width,CvType.CV_8U);
    	mHierarchy = new Mat(height,width,CvType.CV_8U);
    	//hand_bin = Mat.zeros(gray.size(),CvType.CV_8U);
    	MxCtr = new Mat();
    	dist_ind = new Point();
    	pit = new Point();
    	push = false;
    	pushs = false;*/
    	
        matsci=new Mat();
        matpaper=new Mat();
        matstone=new Mat();
        matgun=new Mat();
    }

    public void onCameraViewStopped() {
       // mRgba.release();
    	src.release();
    	mFingers.release();
    }

    public boolean onTouch(View v, MotionEvent event) {

        return false; // don't need subsequent touch events
    }
    
    
    //绘制普通图
    public final Mat overlayImage(Mat background, Mat foreground, Point location)
    {

          Mat mat1Sub= new Mat();  
          if((int)location.x<0)location.x=0; if((int)location.x>background.width()-foreground.width()) location.x=background.width()-foreground.width();
          if((int)location.y<0)location.y=0; if((int)location.y>background.height()-foreground.height()) location.y=background.height()-foreground.height();
          
          //Rect rec = new Rect((int)location.x-foreground.width()/2, (int)location.y-foreground.height()/2, foreground.cols(), foreground.rows());  
          Rect rec = new Rect((int)location.x, (int)location.y, foreground.cols(), foreground.rows());  
          mat1Sub=background.submat(rec);  
          foreground.copyTo(mat1Sub);            
          //source.copyTo(background,mask);
          mat1Sub.release();
          return background;
    }
    
    
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
    	src = inputFrame.rgba();
    	
    	double wd2 = (int) src.size().width;
    	double ht2 = (int) src.size().height;
    	double wd1 = wd2/beishu;
    	double ht1 = ht2/beishu;
    	Size dsize = new Size();
    	dsize.height = (double)ht1;
    	dsize.width = (double)wd1;
    	Imgproc.resize(src, src, dsize);
    	Core.flip(src,src,1);
    	
    	Scalar slr = new Scalar(255,0,0,255);
    	Scalar slrd = new Scalar(0,255,0,255); 
    	Scalar slrdg = new Scalar(255,255,0,255);
    	
    	mFingers.loadSource(src);
    	mFingers.handDetection();
    	
        btsci = BitmapFactory.decodeResource(getResources(),  
                R.drawable.sci);
        btpaper = BitmapFactory.decodeResource(getResources(),  
                R.drawable.paper);
        btstone = BitmapFactory.decodeResource(getResources(),  
                R.drawable.stone);
        btgun = BitmapFactory.decodeResource(getResources(),  
                R.drawable.gun);
//        btpostm1d = BitmapFactory.decodeResource(getResources(),  
//                R.drawable.postm1d);
//        btpostm1u = BitmapFactory.decodeResource(getResources(),  
//                R.drawable.postm1u);
        
        // 转换数据   
        Utils.bitmapToMat(btsci, matsci);  
        Utils.bitmapToMat(btpaper, matpaper); 
        Utils.bitmapToMat(btstone, matstone); 
        Utils.bitmapToMat(btgun, matgun);
//        Utils.bitmapToMat(btplat, matplat); 
//        Utils.bitmapToMat(btpostd1, matpostd1);
    	
    	/*Single Fingertip Detection Demo*/
    	/*mFingers.detectSingleFingerTip();
    	List<Point> Tips0 = mFingers.getHandFingerTipPositions0();
    	Point center0 = mFingers.getCenter0();
    	if(!Tips0.isEmpty())
    	{
    		Core.circle(src,center0,10,slr,-1);
    		Core.circle(src,Tips0.get(0),12,slrd,7);
    		Core.line(src, center0, Tips0.get(0), slrdg,5);
    	}*/
    	
    	
    	/*Push Button Demo*/
    	/*mFingers.detectSingleFingerTip();
    	List<Point> Tips0 = mFingers.getHandFingerTipPositions0();
    	Point center0 = mFingers.getCenter0();
    	if(!Tips0.isEmpty())
    	{
    		Core.circle(src,center0,10,slr,-1);
    		Core.line(src, center0, Tips0.get(0), slrdg,5);
    		if(mFingers.pushButton())
    		{
    			Core.circle(src,Tips0.get(0),12,slrdg,7);
    		}
    		else
    		{
    			Core.circle(src,Tips0.get(0),12,slrd,7);
    		}
    	}*/
    	
    	
    	
    	/*Double Hands Single Fingertips Demo*/
    	/*mFingers.detectDoubleFingerTip();
    	List<Point> Tips0 = mFingers.getHandFingerTipPositions0();
    	Point center0 = mFingers.getCenter0();
    	if(!Tips0.isEmpty())
    	{
    		Core.circle(src,center0,10,slr,-1);
    		Core.circle(src,Tips0.get(0),12,slrd,7);
    		Core.line(src, center0, Tips0.get(0), slrdg,5);
    	}
    	
    	List<Point> Tips1 = mFingers.getHandFingerTipPositions1();
    	Point center1 = mFingers.getCenter1();
    	
    	if(!Tips1.isEmpty())
    	{
    		Core.circle(src,center1,10,slr,-1);
    		Core.circle(src,Tips1.get(0),12,slrd,7);
    		Core.line(src, center1, Tips1.get(0), slrdg,5);
    	}*/
    	
    	
    	/*Multifingertip Demo*/
    	/*mFingers.detectMultiFingerTips();
    	List<Point> Tips0 = mFingers.getHandFingerTipPositions0();
    	Point center0 = mFingers.getCenter0();
    	
    	if(!Tips0.isEmpty())
    	{
    		Core.circle(src,center0,10,slr,-1);
    		for(int i=0;i<Tips0.size();i++)
    		{
    			Core.circle(src,Tips0.get(i),12,slrd,7);
    			Core.line(src, center0, Tips0.get(i), slrdg,5);
    		}
    	}*/
    	
    	/*Gesture Recognition*/
    	mFingers.detectMultiFingerTips();
    	List<Point> Tips0 = mFingers.getHandFingerTipPositions0();
    	Point center0 = mFingers.getCenter0();
    	//float[] dire = mFingers.getDirection();
    	
    	exist=mFingers.getExistence0();
    	category=mFingers.getCategory();
    			
/*    	if(exist)
    	{
    		//Point start = new Point(center0.x-50*(double)dire[0],center0.y-50*(double)dire[1]);
        	//Point end = new Point(center0.x+50*(double)dire[0],center0.y+50*(double)dire[1]);
    		//Core.circle(src,center0,10,slr,-1);
    		Point centerText = new Point(center0.x-50,center0.y);
    		switch(category)
    		{
    		case 0: 
    			{Core.putText(src, "Hammer", centerText, 0, 2, slrdg,3);
    		     break;
    			}
    		case 1: Core.putText(src, "Index", centerText, 0, 2, slrdg,3); break;
    		case 2: Core.putText(src, "Scissor", centerText, 0, 2, slrdg,3); break;
    		case 3: Core.putText(src, "Cloth", centerText, 0, 2, slrdg,3); break;
    		case 4: Core.putText(src, "Gun", centerText, 0, 2, slrdg,3); break;
    		case 5: Core.putText(src, "Cloth", centerText, 0, 2, slrdg,3); break;
    		default: break;
    		}
    		for(int i=0;i<Tips0.size();i++)
    		{
    			//Core.line(src, center0, Tips0.get(i), slrdg,5);
    			//Core.line(src, start, end, slrdg,5);
    			//Core.putText(src, Double.toString(mFingers.getGesture()), center0, 0, 5, slrdg);
    			Core.circle(src,Tips0.get(i),12,slrd,7);
    		}
    	}*/
    	
   	
        Size dsized = new Size();
        dsized.height = ht2;
        dsized.width = wd2;
        Imgproc.resize(src, src, dsized);
    	
        
    	if(exist)
    	{
    		//Point start = new Point(center0.x-50*(double)dire[0],center0.y-50*(double)dire[1]);
        	//Point end = new Point(center0.x+50*(double)dire[0],center0.y+50*(double)dire[1]);
    		//Core.circle(src,center0,10,slr,-1);
    		Point centerText = new Point(center0.x+150,center0.y+200);
    		switch(category)
    		{
    		case 0: 
    			{Core.putText(src, "Stone", centerText, 0, 6, slrdg,9);
    			 src=overlayImage(src,matstone,new Point(1050,20));
    		     break;
    			}
    		case 1: //Core.putText(src, "Index", centerText, 0, 26, slrdg,29); 
    			break;
    		case 2: 
			{Core.putText(src, "Scissors", centerText, 0, 6, slrdg,9);
			 src=overlayImage(src,matsci,new Point(1050,20));
		     break;
			}
    		case 3: 
			{Core.putText(src, "Paper", centerText, 0, 6, slrdg,9);
			 src=overlayImage(src,matpaper,new Point(1050,20));
		     break;
			}
			
    		case 4: 
			{Core.putText(src, "Gun", centerText, 0, 6, slrdg,9);
			 src=overlayImage(src,matgun,new Point(1050,20));
		     break;
			}
    		case 5: 
			{Core.putText(src, "Paper", centerText, 0, 6, slrdg,9);
			 src=overlayImage(src,matpaper,new Point(1050,20));
		     break;
			}
    		default: break;
    		}
/*    		for(int i=0;i<Tips0.size();i++)
    		{
    			//Core.line(src, center0, Tips0.get(i), slrdg,5);
    			//Core.line(src, start, end, slrdg,5);
    			//Core.putText(src, Double.toString(mFingers.getGesture()), center0, 0, 5, slrdg);
    			Core.circle(src,new Point(Tips0.get(i).x*beishu,Tips0.get(i).y*beishu),30,slrd,17);
    		}*/
    	}
    	//test
    	//double mm=mFingers.getGesture();
    	//Core.putText(src,Double.toString(mm), new Point(600,200),1 , 3.0f, new Scalar(255,0,255,255),5);
    	
    	mFingers.reinitialize();
    	
    	return src;
    }
   /* private double Dist_Calc(Point pt1, Point pt2)
    {
    	double dist;
    	return(Math.sqrt((pt1.x-pt2.x)*(pt1.x-pt2.x)+(pt1.y-pt2.y)*(pt1.y-pt2.y)));
    }
   private double Dot_Calc(Point ptp, Point ptd, Point pt)
    {
    	return((ptp.x-pt.x)*(ptd.x-pt.x)+(ptp.y-pt.y)*(ptd.y-pt.y));
    }
    private double Costheta_Calc(Point ptp, Point ptd, Point pt)
    {
    	return(Dot_Calc(ptp,ptd,pt)/(Dist_Calc(ptp,pt)*Dist_Calc(ptd,pt)));
    }
    private Point Farthest_Point(Point center,Point[] pnt,float dire[])
    {
    	double maxDist = 0;
    	Point ptc = new Point(center.x+dire[0],center.y+dire[1]);
        Point fgr = new Point(0,0);
        for(int i=0;i<pnt.length;i++)
        {
        	if(Dist_Calc(pnt[i],center)*(1+0.5*Math.abs(Costheta_Calc(pnt[i],ptc,center)))>maxDist)
        	{
        		maxDist = Dist_Calc(pnt[i],center)*(1+0.5*Math.abs(Costheta_Calc(pnt[i],ptc,center)));
        		fgr = pnt[i];
        	}
        }
    	return fgr;
    }
    private float[] PCACalc(Mat binary_img)
    {
    	Moments mmt = Imgproc.moments(binary_img);
    	//double mu00 = mmt.get_m00();
    	double m00 = mmt.get_m00();
    	double m01 = mmt.get_m01();
    	double m02 = mmt.get_m02();
    	double m10 = mmt.get_m10();
    	double m20 = mmt.get_m20();
    	double m11 = mmt.get_m11();
    	
    	float varx = (float)(m20/m00 - (m10/m00)*(m10/m00));
    	float vary = (float)(m02/m00 - (m01/m00)*(m01/m00));
    	float covxy = (float)(m11/m00 - m10*m01/(m00*m00));
    	
    	Mat covmtr = new Mat(2,2,CvType.CV_32FC1);
    	
    	float[] data = {varx};
    	covmtr.put(0, 0, data);
    	data[0] = covxy;
    	covmtr.put(0, 1, data);
    	covmtr.put(1, 0, data);
    	data[0] = vary;
    	covmtr.put(1, 1, data);
    	
    	Mat eigenvalues = new Mat();
    	Mat eigenvectors =  new Mat();
    	Core.eigen(covmtr, true, eigenvalues, eigenvectors);
    	
    	
    	float[] orit = new float[2];
    	eigenvectors.get(0, 0, data);
    	orit[0] = data[0];
    	eigenvectors.get(0, 1, data);
    	orit[1] = data[0];
    	
    	//normalize
    	double mode = Math.sqrt(orit[0]*orit[0]+orit[1]*orit[1]);
    	orit[0] = (float) (orit[0]/mode);
    	orit[1] = (float) (orit[1]/mode);
    	
    	return orit;
    }
    /*private boolean Push_Button(Point center,Point[] pnt, int[] Dhull, double dist_th, Point curse)
    {
    	boolean SF = true;
    	for(int i=0;i<Dhull.length;i+=4)
        {
    		int pre = Dhull[i];
    		int dft = Dhull[i+2];
    		int aft = Dhull[i+1];
    		Point pt = pnt[dft];
    		Point ptd = pnt[aft];
    		Point ptp = pnt[pre];
    		if(((Dist_Calc(center,ptd)/(Dist_Calc(center,pt))>1.3))||(Dist_Calc(center,ptp)/Dist_Calc(center,pt)>1.3))
    		{
    			if(Costheta_Calc(ptp,ptd,pt)>-0.3)
    			{
    				if((Dist_Calc(ptd,center)>dist_th)&&(Dist_Calc(ptp,center)>dist_th))
    				{
    					if((Costheta_Calc(ptp,curse,center)>-0.2)&&((Costheta_Calc(ptd,curse,center)>-0.2)))
    					{
    						SF = false;
    					}
    				}
    			}
    		}
        }
    	
    	return SF;
    }*/
    /*private List<Point> Defect_Classifier(Point center,Point[] pnt, int[] Dhull, double dist_th)
    {
    	//Array to return
    	List<Point> tips = new ArrayList<Point>();
    	//Intermediate variables
    	List<Integer> idx = new ArrayList<Integer>();
    	//First run of classification
    	for(int i=0;i<Dhull.length;i+=4)
        {
    		int pre = Dhull[i];
    		int dft = Dhull[i+2];
    		int aft = Dhull[i+1];
    		Point pt = pnt[dft];
    		Point ptd = pnt[aft];
    		Point ptp = pnt[pre];
    		if(((Dist_Calc(center,ptd)/(Dist_Calc(center,pt))>1.2))||(Dist_Calc(center,ptp)/Dist_Calc(center,pt)>1.2))
    		{
    			if(Costheta_Calc(ptp,ptd,pt)>-0.1)
    			{
    				if((Dist_Calc(ptd,center)>dist_th)&&(Dist_Calc(ptp,center)>dist_th))
    				{
    					idx.add(pre);
    					idx.add(aft);
    				}
    			}
    		}
        }
    	//Second run of classification
    	int num=0;
    	while(num<idx.size())
    	{
    		if((num%2!=0)&&(num<idx.size()-1))
            {
    			if(idx.get(num+1)-idx.get(num)<25)
                {
    				tips.add(pnt[(int)((idx.get(num+1)+idx.get(num))/2)]);
                    num = num + 2;
                }
                else
                {
                	tips.add(pnt[idx.get(num)]);
                    num++;
                }
            }
            else
            {
            	tips.add(pnt[idx.get(num)]);
                num++;
            }

    	}
    	return tips;
    }*/
    /*private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }*/
}