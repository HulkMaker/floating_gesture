package org.opencv.samples.colorblobdetect;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
//import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.video.KalmanFilter;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
//import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;

public class FingerTipDetection {
	/*Source and Intermediate Variables*/
	//Mats
	private Mat mSourceFrame;
	private Mat mSkinArea;
	private Mat mHandBin0;
	private Mat mHandBin1;
	private Mat mDistance0;
	private Mat mDistance1;
	private Mat mShowImage;
	
	//Fingertips Lists
	private List<Point> lFingerTips0;
	private List<Point> lFingerTips1;
	
	//Finger Centers
	private Point pHandCenter0;
	private Point pHandCenter1;
	
	//Contours
	private List<MatOfPoint> lContours;
	private MatOfPoint mpHandContour0;
	private MatOfPoint mpHandContour1;
	private Point[] pHandContour0;
	private Point[] pHandContour1;
	
	//If hand exists
	private boolean bHandExists;
	private boolean bTwoHandExist;
	
	/*Tool Variables*/
	private int iContourIndex0;
	private int iContourIndex1;
	
	private Mat mSourceFrameInYCbCr;
	private Mat mBinR;
	private Mat mBinB;
	private Mat mBinRLow;
    private Mat mBinRHigh;
    private Mat mBinRBin;
    private Mat mBinBLow;
    private Mat mBinBHigh;
    private Mat mBinBBin;
    
    private List<Mat> lColorChannels;
    private int iGesture;
    private double dGesture;
    
    /*Tool Functions*/
    //Dot Calculation
    private double tDotCalculation(Point ptp, Point ptd, Point pt)
    {
    	return((ptp.x-pt.x)*(ptd.x-pt.x)+(ptp.y-pt.y)*(ptd.y-pt.y));
    }
    //Distance Calculation
    private double tDistanceCalculation(Point pt1, Point pt2)
    {
    	return(Math.sqrt((pt1.x-pt2.x)*(pt1.x-pt2.x)+(pt1.y-pt2.y)*(pt1.y-pt2.y)));
    }
	//Cosine Calculation
    private double tCosineCalculation(Point ptp, Point ptd, Point pt)
    {
    	return(tDotCalculation(ptp,ptd,pt)/(tDistanceCalculation(ptp,pt)*tDistanceCalculation(ptd,pt)));
    }
    //Farthest Point Calculation
    private Point tFarthestPoint(Point center,Point[] pnt,float[] dire)
    {
    	double maxDist = 0;
    	Point ptc = new Point(center.x+dire[0],center.y+dire[1]);
        Point fgr = new Point(0,0);
        for(int i=0;i<pnt.length;i++)
        {
        	if(tDistanceCalculation(pnt[i],center)*(1+0.5*Math.abs(tCosineCalculation(pnt[i],ptc,center)))>maxDist)
        	{
        		maxDist = tDistanceCalculation(pnt[i],center)*(1+0.5*Math.abs(tCosineCalculation(pnt[i],ptc,center)));
        		fgr = pnt[i];
        	}
        }
    	return fgr;
    }
    //PCA Calculation
    private float[] tCalculatePrincipalComponent(Mat binary_img)
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
    private Point getCenter(Mat binary_img)
    {
    	Moments moment = Imgproc.moments(binary_img);
    	Point pit = new Point(moment.get_m10()/moment.get_m00(),moment.get_m01()/moment.get_m00());
    	return pit;
    }
    
    /*Module Function*/
    //Get Binary Image
    public void getSkinArea()
    {
    	Imgproc.cvtColor(mSourceFrame, mSourceFrameInYCbCr, Imgproc.COLOR_BGR2YCrCb);
    	Core.split(mSourceFrameInYCbCr, lColorChannels);
    	mBinR = lColorChannels.get(1);
    	mBinB = lColorChannels.get(2);
    	
    	Imgproc.threshold(mBinR, mBinRLow, 92, 255, Imgproc.THRESH_BINARY);
    	Imgproc.threshold(mBinR, mBinRHigh, 130, 255, Imgproc.THRESH_BINARY_INV);
    	Imgproc.threshold(mBinB, mBinBLow, 133, 255, Imgproc.THRESH_BINARY);
    	Imgproc.threshold(mBinB, mBinBHigh, 170, 255, Imgproc.THRESH_BINARY_INV);
    	Core.bitwise_and(mBinRLow, mBinRHigh, mBinRBin);
    	Core.bitwise_and(mBinBLow, mBinBHigh, mBinBBin);
    	Core.bitwise_and(mBinRBin, mBinBBin, mSkinArea);
    }
    //Check if Hand Exists
    public void checkHandExistance()
    {
    	Mat mHierarchy = new Mat();
    	Imgproc.findContours(mSkinArea, lContours, mHierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
    	double maxArea = 0;
    	double secondMaxArea = 0;
    	int count = 0;
        Iterator<MatOfPoint> each = lContours.iterator();
        //Find the largest and the second largest contour
        while (each.hasNext()) 
        {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if(area > maxArea)
            {
            	secondMaxArea = maxArea;
            	mHandBin1 = mHandBin0;
            	iContourIndex1 = iContourIndex0;
            	
            	maxArea = area;
            	mpHandContour0 = wrapper;
            	iContourIndex0 = count;
            }
            else if(area > secondMaxArea)
            {
            	secondMaxArea = area;
            	mpHandContour1 = wrapper;
            	iContourIndex1 = count;
            }
            count++;
        }
        //draw contours on separate canvas
        //Judge if there exists no hand, one hand or two hands
        if(maxArea>1600)
        {
        	mHandBin0.create(mSkinArea.size(), CvType.CV_8U);
        	bHandExists = true;
        	Imgproc.drawContours(mHandBin0, lContours, iContourIndex0, new Scalar(255), -1);
        	Imgproc.threshold(mHandBin0, mHandBin0, 254, 255, Imgproc.THRESH_BINARY);
        	pHandCenter0 = getCenter(mHandBin0);
        	if(secondMaxArea>1200)
        	{
        		mHandBin1.create(mSkinArea.size(), CvType.CV_8U);
        		bTwoHandExist = true;
        		Imgproc.drawContours(mHandBin1, lContours, iContourIndex1, new Scalar(255), -1);
        		Imgproc.threshold(mHandBin1, mHandBin1, 254, 255, Imgproc.THRESH_BINARY);
        		pHandCenter1 = getCenter(mHandBin1);
        	}
        }
    }
    
    /*Processing Function*/
    //initialize state variables
    public void initialize()
    {
    	//initialize state variables
    	mSourceFrame =  new Mat();
    	mSkinArea = new Mat();
    	mHandBin0 = new Mat();
    	mHandBin1 = new Mat();
    	mDistance0 = new Mat();
    	mDistance1 = new Mat();
    	mShowImage = new Mat();
    	
    	lFingerTips0 = new ArrayList<Point>();
    	lFingerTips1 = new ArrayList<Point>();
    	
    	pHandCenter0 = new Point();
    	pHandCenter1 = new Point();
    	
    	lContours = new ArrayList<MatOfPoint>();
    	mpHandContour0 = new MatOfPoint();
    	mpHandContour1 = new MatOfPoint();
    	
    	bHandExists = false;
    	bTwoHandExist = false;
    	
    	//initialize tool variables
    	iContourIndex0 = 0;
    	iContourIndex1 = 0;
    	
    	mSourceFrameInYCbCr = new Mat();
    	mBinR = new Mat();
    	mBinB = new Mat();
    	mBinRLow = new Mat();
        mBinRHigh = new Mat();
        mBinRBin = new Mat();
        mBinBLow = new Mat();
        mBinBHigh = new Mat();
        mBinBBin = new Mat();
        
        lColorChannels = new ArrayList<Mat>();
    }
    //reinitialize the state
    public void reinitialize()
    {
    	lFingerTips0.clear();
    	lFingerTips1.clear();
    	
    	bHandExists = false;
    	bTwoHandExist = false;
    	
    	pHandCenter0 = new Point();
    	pHandCenter1 = new Point();
    	
    	mHandBin0 = new Mat();
    	mHandBin1 = new Mat();
    	
    	lColorChannels.clear();
    	lContours.clear();
    }
    //Load Source Image
    public void loadSource(Mat inputFrame)
    {
    	mSourceFrame.create(inputFrame.size(), CvType.CV_8UC4);
    	inputFrame.copyTo(mSourceFrame);
    }
    //Detect Hands
    public void handDetection()
    {
    	getSkinArea();
    	checkHandExistance();
    	//pHandContour0 = mpHandContour0.toArray();
    	//pHandContour1 = mpHandContour1.toArray();
    }
    //Single Fingertip Detection
    public void detectSingleFingerTip()
    {
    	if(bHandExists)
    	{
    		pHandContour0 = mpHandContour0.toArray();
        	Imgproc.distanceTransform(mHandBin0, mDistance0, Imgproc.CV_DIST_L2, 3);
        	float[] dire = tCalculatePrincipalComponent(mDistance0);
        	lFingerTips0.add(tFarthestPoint(pHandCenter0,pHandContour0,dire));
    	}
    }
    //FingerTipDetection For Two Hands
    public void detectDoubleFingerTip()
    {
    	if(bHandExists)
    	{
    		pHandContour0 = mpHandContour0.toArray();
        	Imgproc.distanceTransform(mHandBin0, mDistance0, Imgproc.CV_DIST_L2, 3);
        	float[] dire0 = tCalculatePrincipalComponent(mDistance0);
        	lFingerTips0.add(tFarthestPoint(pHandCenter0,pHandContour0,dire0));
        	if(bTwoHandExist)
        	{
        		pHandContour1 = mpHandContour1.toArray();
            	Imgproc.distanceTransform(mHandBin1, mDistance1, Imgproc.CV_DIST_L2, 3);
            	float[] dire1 = tCalculatePrincipalComponent(mDistance1);
            	lFingerTips1.add(tFarthestPoint(pHandCenter1,pHandContour1,dire1));
        	}
    	}
    }
    //Push Button
    public boolean pushButton()
    {
    	boolean bSingleFinger = true;
    	MatOfInt miChull = new MatOfInt();
    	MatOfInt4 miDhull = new MatOfInt4();
    	Imgproc.convexHull(mpHandContour0, miChull);
    	Imgproc.convexityDefects(mpHandContour0, miChull, miDhull);
    	Point curse = lFingerTips0.get(0);
    	double dist_th = tDistanceCalculation(pHandCenter0, curse)*0.7;
    	int[] Dhull = miDhull.toArray();
    	for(int i=0;i<Dhull.length;i+=4)
        {
    		int pre = Dhull[i];
    		int dft = Dhull[i+2];
    		int aft = Dhull[i+1];
    		Point pt = pHandContour0[dft];
    		Point ptd = pHandContour0[aft];
    		Point ptp = pHandContour0[pre];
    		if(((tDistanceCalculation(pHandCenter0, ptd)/(tDistanceCalculation(pHandCenter0,pt))>1.3))||(tDistanceCalculation(pHandCenter0,ptp)/tDistanceCalculation(pHandCenter0,pt)>1.3))
    		{
    			if(tCosineCalculation(ptp,ptd,pt)>-0.3)
    			{
    				if((tDistanceCalculation(ptd,pHandCenter0)>dist_th)&&(tDistanceCalculation(ptp,pHandCenter0)>dist_th))
    				{
    					if((tCosineCalculation(ptp,curse,pHandCenter0)>-0.2)&&((tCosineCalculation(ptd,curse,pHandCenter0)>-0.2)))
    					{
    						bSingleFinger = false;
    					}
    				}
    			}
    		}
        }
    	
    	return bSingleFinger;
    }
    //Multifinger Detection
    public void detectMultiFingerTips()
    {
    	if(bHandExists)
    	{
    		//Intermediate variables
        	List<Integer> idx = new ArrayList<Integer>();
        	//First run of classification
        	pHandContour0 = mpHandContour0.toArray();
        	Imgproc.distanceTransform(mHandBin0, mDistance0, Imgproc.CV_DIST_L2, 3);
        	float[] dire = tCalculatePrincipalComponent(mDistance0);
        	Point curse = tFarthestPoint(pHandCenter0,pHandContour0,dire);
        	MatOfInt miChull = new MatOfInt();
        	MatOfInt4 miDhull = new MatOfInt4();
        	Imgproc.convexHull(mpHandContour0, miChull);
        	Imgproc.convexityDefects(mpHandContour0, miChull, miDhull);
        	double dist_th = tDistanceCalculation(pHandCenter0, curse)*0.7;
        	int[] Dhull = miDhull.toArray();
        	for(int i=0;i<Dhull.length;i+=4)
            {
        		int pre = Dhull[i];
        		int dft = Dhull[i+2];
        		int aft = Dhull[i+1];
        		Point pt = pHandContour0[dft];
        		Point ptd = pHandContour0[aft];
        		Point ptp = pHandContour0[pre];
        		if(((tDistanceCalculation(pHandCenter0,ptd)/(tDistanceCalculation(pHandCenter0,pt))>1.2))||(tDistanceCalculation(pHandCenter0,ptp)/tDistanceCalculation(pHandCenter0,pt)>1.2))
        		{
        			if(tCosineCalculation(ptp,ptd,pt)>-0.1)
        			{
        				if((tDistanceCalculation(ptd,pHandCenter0)>dist_th)&&(tDistanceCalculation(ptp,pHandCenter0)>dist_th))
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
        				lFingerTips0.add(pHandContour0[(int)((idx.get(num+1)+idx.get(num))/2)]);
                        num = num + 2;
                    }
                    else
                    {
                    	lFingerTips0.add(pHandContour0[idx.get(num)]);
                        num++;
                    }
                }
                else
                {
                	lFingerTips0.add(pHandContour0[idx.get(num)]);
                    num++;
                }

        	}
        	//Third run of classification
        	int sum = 0;
        	List<Integer> candidate = new ArrayList<Integer>();
        	Point ptc = new Point(pHandCenter0.x+dire[0],pHandCenter0.y+dire[1]);
        	for(int i=0;i<lFingerTips0.size();i++)
        	{
        		if(tCosineCalculation(lFingerTips0.get(i),ptc,pHandCenter0)>0)
        		{
        			sum++;
        			candidate.add(1);
        		}
        		else
        		{
        			sum--;
        			candidate.add(-1);
        		}
        	}
        	Iterator<Point> iter = lFingerTips0.iterator();
        	Iterator<Integer> itcd = candidate.iterator();
        	while(iter.hasNext()&&itcd.hasNext())
        	{   
        	    Point cdp = iter.next();
        	    int cdi = itcd.next();
        	    if(sum*cdi<0)
        	    {
        	    	itcd.remove();
        	    	iter.remove();
        	    }
                    //CAUTION!!!!!!!!
                    else if(sum*cdi==0)
                    {
                        if(tCosineCalculation(cdp, curse, pHandCenter0)<0.9)
                        {
                            itcd.remove();
                            iter.remove();
                         }
                     }
                    //END CAUTION
        	}
        	//Gesture Recognition
        	dGesture = 0;
        	int numFingers = lFingerTips0.size();
        	for(int i=0;i<lFingerTips0.size();i++)
        	{
        		//dGesture = dGesture+Math.abs(tCosineCalculation(lFingerTips0.get(i),ptc,pHandCenter0));
                       //dGesture = dGesture+Math.abs(tCosineCalculation(lFingerTips0.get(i),curse,pHandCenter0));  //CAUTION! Using the method here, don't forget to change the following
                        dGesture = dGesture+Math.abs(tCosineCalculation(lFingerTips0.get(i),curse,pHandCenter0))+Math.abs(tCosineCalculation(lFingerTips0.get(i),ptc,pHandCenter0));  //CAUTION! Using the method here, don't forget to change the following
        	}
        	switch(numFingers)
        	{
        	case 0: iGesture = 0; break;
        	case 1: iGesture = 1; break;
        	case 2: iGesture = (dGesture>0.75)? 2:4; break; //CAUTION! Change the threshold here to make the result more accurate(default=1.8).
        	case 3: iGesture = 3; break;
        	default:iGesture = 5; break;
        	}
    	}
    }
    //release memory
    public void release()
    {
    	mSourceFrame.release();
    	mSkinArea.release();
    	mHandBin0.release();
    	mHandBin1.release();
    	mDistance0.release();
    	mDistance1.release();
    	mShowImage.release();
    	
    	lFingerTips0.clear();
    	lFingerTips1.clear();
    	
    	
    	lContours.clear();
    	mpHandContour0.release();
    	mpHandContour1.release();
    	
    	mSourceFrameInYCbCr.release();
    	mBinR.release();
    	mBinB.release();
    	mBinRLow.release();
        mBinRHigh.release();
        mBinRBin.release();
        mBinBLow.release();
        mBinBHigh.release();
        mBinBBin.release();
        
        lColorChannels.clear();;
    }
    
    /*Interface Functions*/
   public List<Point> getHandFingerTipPositions0()
   {
	   return lFingerTips0;
   }
   public List<Point> getHandFingerTipPositions1()
   {
	   return lFingerTips1;
   }
   public Point getCenter0()
   {
	   return pHandCenter0;
   }
   public Point getCenter1()
   {
	   return pHandCenter1;
   }
   public double getGesture()
   {
           //CAUTION
	   return dGesture;
   }
   public int getCategory()
   {
	   return iGesture;
   }
   public boolean getExistence0()
   {
	   return bHandExists;
   }
   /*public Mat getShowImage(boolean mask)
   {
	   if(mask)
	   {
		   mSourceFrame.copyTo(mShowImage,mSkinArea);
	   }
	   else
	   {
		   mSourceFrame.copyTo(mShowImage);
	   }
	   return mShowImage;
   }*/
    

}
