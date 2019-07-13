//
//  NostalgiaCamera.m
//  PosenetOnIOS
//
//  Created by Dwayne Forde on 2017-12-23.
//

#import <opencv2/opencv.hpp>
#import <opencv2/videoio/cap_ios.h>
#import <opencv2/imgcodecs/ios.h>
#include "NostalgiaCamera.h"
#include "TfliteWrapper.h"
#include "PoseParse.hpp"

using namespace std;
using namespace cv;


@interface NostalgiaCamera () <CvVideoCameraDelegate>
@end


@implementation NostalgiaCamera
{
    UIViewController<NostalgiaCameraDelegate> * delegate;
    UIImageView * imageView;
    CvVideoCamera * videoCamera;
    TfliteWrapper  *tfLiteWrapper;
    CDecodePose *pDecodePose;
    float minPoseConfidence;
    float minPartConfidence;
    int maxPoseDection;
    map<int, map<int, vector<int> > > result;
}

- (id)initWithController:(UIViewController<NostalgiaCameraDelegate>*)c andImageView:(UIImageView*)iv
{
    delegate = c;
    imageView = iv;
    
    videoCamera = [[CvVideoCamera alloc] initWithParentView:imageView]; // Init with the UIImageView from the ViewController
    videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack; // Use the back camera
    videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait; // Ensure proper orientation
    videoCamera.rotateVideo = YES; // Ensure proper orientation
    videoCamera.defaultFPS = 30; // How often 'processImage' is called, adjust based on the amount/complexity of images
    videoCamera.delegate = self;
    //videoCamera.recordVideo = YES;
    
    tfLiteWrapper = [[TfliteWrapper alloc]init];
    tfLiteWrapper = [tfLiteWrapper initWithModelFileName:@"multi_person_mobilenet_v1_075_float"];
    if(![tfLiteWrapper setUpModelAndInterpreter])
    {
        NSLog(@"Failed To Load Model");
        return self;
    }
    
    minPoseConfidence = 0.5;
    minPartConfidence = 0.1;
    maxPoseDection = 5;
    pDecodePose = new CDecodePose();

    return self;
}

- (void)processImage:(cv::Mat &)frame {
    cv::Mat small;
    int height = frame.rows;
    int width = frame.cols;
    cv::resize(frame, small, cv::Size(pDecodePose->m_inputWidth, pDecodePose->m_inputHeight), 0, 0, CV_INTER_LINEAR);
    float_t *input = [tfLiteWrapper inputTensortFloatAtIndex:0];
    //NSLog(@"Input: %f", *input);
    
    //BGRA2RGB
    int inputCnt=0;
    for (int row = 0; row < small.rows; row++)
    {
        uchar* data = small.ptr(row);
        for (int col = 0; col < small.cols; col++)
        {
            input[inputCnt++] = (float)data[col * 4 + 2]/255.0; // Red
            input[inputCnt++] = (float)data[col * 4 + 1]/255.0; // Green
            input[inputCnt++] = (float)data[col * 4 ]/255.0; // Bule
        }
    }
    
    if([tfLiteWrapper invokeInterpreter])
    {
        result.clear();
        float_t *score = [tfLiteWrapper outputTensorAtIndex:0];
        float_t *shortOffset  = [tfLiteWrapper outputTensorAtIndex:1];
        float_t *middleOffset = [tfLiteWrapper outputTensorAtIndex:2];
        pDecodePose->decode(score, shortOffset, middleOffset, result);
    }
    
    int poseCnt = 0;
    map<int, map<int, vector<int> > >::reverse_iterator it;
    for(it = result.rbegin(); it != result.rend(); ++it)
    {
        if(poseCnt++ > maxPoseDection)
            break;
        
        if(it->first/INT_TO_FLOAT < minPoseConfidence)
            break;
        
        NSLog(@"Output: score [%d]", it->first);
        
        if(it->second.size() != pDecodePose->m_kCnt)
        {
            NSLog(@"Warning: Pose Count[%lu] ！= 17", it->second.size() );
            continue;
        }
        
        for(int i=0; i< it->second.size(); i++)
        {
            float score = it->second[i][5]/INT_TO_FLOAT;
            if(score < minPartConfidence)
                continue;
            
            int pointH = int((it->second[i][6]/10000.0) * height);
            int pointW = int((it->second[i][7]/10000.0) * width);
            cv::Point center = cv::Point(pointW, pointH);
            cv::circle(frame, center, 5, (0,255,255));
        }
        
        for(int i=0; i < pDecodePose->m_eCnt; i++)
        {
            int srcKeypoint = pDecodePose->m_childOrder[i];
            int tagKeypoint = pDecodePose->m_parentOrder[i];
            float srcScore = it->second[srcKeypoint][5]/INT_TO_FLOAT;
            float tagScore = it->second[tagKeypoint][5]/INT_TO_FLOAT;
            if(srcScore < minPartConfidence || tagScore < minPartConfidence)
                continue;
            
            int srcH = int((it->second[srcKeypoint][6]/INT_TO_FLOAT) * height);
            int srcW = int((it->second[srcKeypoint][7]/INT_TO_FLOAT) * width);
            int tagH = int((it->second[tagKeypoint][6]/INT_TO_FLOAT) * height);
            int tagW = int((it->second[tagKeypoint][7]/INT_TO_FLOAT) * width);

            cv::line(frame,cv::Point(srcW, srcH), cv::Point(tagW, tagH), (255,0,255));
            //NSLog(@"Output: [%d] [%d] -> [%d] [%d]", srcH, srcW, tagH, tagW);
        }
    }
    NSLog(@"Output: ------");
    return;
}

- (void)start
{
    [videoCamera start];
}

- (void)stop
{
    [videoCamera stop];
}

@end
