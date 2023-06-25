#include <Windows.h>
#include <iostream>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>
//#include <Ole2.h>
#include<fstream>

#define Width 640
#define Height 480


using namespace std;
using namespace cv;

// Define event handle
// Create the next frame event handle to determine if Kinect can start to read the next frame data
HANDLE nextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
HANDLE colorStreamHandle = NULL; // The handle used to store the color image data stream
HANDLE nextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
HANDLE depthStreamHandle = NULL; // The handle used to store the depth image data stream

long depthToRgbMap[Width * Height * 2];
float depthData[Width * Height * 3];
string sss = "2";

string colorImgPath = "colorImg"+sss+".jpg"; //Path to save the color img
string depthImgPath = "depthImg"+sss+".png"; //Path to save the depth img
string registeredRgbImgPath = "registeredRgbImg"+sss+".jpg";
string pcdPath = "test_pcd"+sss+".txt";

string colorImgPath_t= "colorImg_t.jpg";//For callibration
string depthImgPath_t = "depthImg_t.png";
string registeredRgbImgPath_t = "registeredRgbImg_t.jpg";
string pcdPath_t = "test_pcd_t.txt";

bool initKinect() {
    HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
    if (FAILED(hr))
    {
        cout << "NuiIntialize failed" << endl;
        return FALSE;
    }

    //  Open Kinect's color image channel，use colorStreamHandle to store the data
    hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,// Depth camera or rgb camera?
        NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
        0,        // Image stream flags, e.g. near mode
        2,        // Number of frames to buffer
        nextColorFrameEvent,     // Event handle
        &colorStreamHandle);
    if (FAILED(hr))
    {
        cout << "Could not open color image stream video" << endl;
        NuiShutdown();
        return FALSE;
    }
    // The same for depth image
    hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480, 0, 2,
        nextDepthFrameEvent, &depthStreamHandle);
    if (FAILED(hr))
    {
        cout << "Could not open depth image stream video" << endl;
        NuiShutdown();
        return FALSE;
    }
    return TRUE;
}

// To write the point cloud data into a txt file
int writeTXT(float* depthData, cv::Mat rgbData, string pcdPath)
{    
    float* curr = (float*)depthData;
    ofstream OutFile(pcdPath);
    for (int i = 0; i < rgbData.rows; i++)
    {
        uchar* ptr = rgbData.ptr<uchar>(i);
        for (int j = 0; j < rgbData.cols; j++)
        {
            OutFile << *curr++ << " ";
            OutFile << *curr++ << " ";
            OutFile << *curr++ << " ";
            OutFile << (float)ptr[2] / 255.f << " ";
            OutFile << (float)ptr[1] / 255.f << " ";
            OutFile << (float)ptr[0] / 255.f << endl;
            ptr += 3;
        }
    }
    OutFile.close();
    return 0;
}

int getDepth(cv::Mat DepthImg)
{
    const NUI_IMAGE_FRAME* pDepthImageFrame = NULL;
    if (FAILED(NuiImageStreamGetNextFrame(depthStreamHandle, 0, &pDepthImageFrame)))
    {
        cout << "Could not get depth image" << endl;
        NuiShutdown();
        return 0;
    }
    // Extract frame to LockedRect, it contains two data objects: pitch is the number of bytes per line; pBits is the address of firts byte
    // We lock the data so that kinect won't modify it when we read
    INuiFrameTexture* pDepthTexture = pDepthImageFrame->pFrameTexture;
    NUI_LOCKED_RECT depthLockedRect;
    pDepthTexture->LockRect(0, &depthLockedRect, NULL, 0);
    if (depthLockedRect.Pitch != 0)
    {
        long* depth2rgb = (long*)depthToRgbMap;
        float* fdest = (float*)depthData;
        for (int i = 0; i < DepthImg.rows; i++)
        {
            uchar* ptr = DepthImg.ptr<uchar>(i);

            // Depth image use 2 bytes for each pixel and only lower 12 bits are used
            uchar* pBufferRun = (uchar*)(depthLockedRect.pBits) + i * depthLockedRect.Pitch;
            USHORT* pBuffer = (USHORT*)pBufferRun;

            for (int j = 0; j < DepthImg.cols; j++)
            {
                ptr[j] = (uchar)(255 * pBuffer[j] / 0x0fff);    // normalization


              // Store coordinates of the point corresponding to pixel on depth image
                USHORT depth = NuiDepthPixelToDepth(pBuffer[j]);
                Vector4 pos = NuiTransformDepthImageToSkeleton(j, i, depth << 3, NUI_IMAGE_RESOLUTION_640x480);
                *fdest++ = pos.x / pos.w;
                *fdest++ = pos.y / pos.w;
                *fdest++ = pos.z / pos.w;
                // Store the index into the color array corresponding to this pixel
                NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
                    NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL,
                    j, i, depth << 3, depth2rgb, depth2rgb + 1);
                depth2rgb += 2;
            }
        }
    }
    else
    {
        cout << "Buffer length of received texture is bogus\r\n" << endl;
    }
    pDepthTexture->UnlockRect(0);// Finished, unlock and release the frame for updating
    NuiImageStreamReleaseFrame(depthStreamHandle, pDepthImageFrame);
    return 1;
}

int getColor(cv::Mat ColorImg, cv::Mat RgbData)
{
    const NUI_IMAGE_FRAME* pColorImageFrame = NULL;
    //Get the color img
    if (FAILED(NuiImageStreamGetNextFrame(colorStreamHandle, 0, &pColorImageFrame)))
    {
        cout << "Could not get color image" << endl;
        NuiShutdown();
        return 0;
    }
    INuiFrameTexture* pColorTexture = pColorImageFrame->pFrameTexture;
    NUI_LOCKED_RECT colorLockedRect;
    pColorTexture->LockRect(0, &colorLockedRect, NULL, 0);

    // check if the data is valid
    if (colorLockedRect.Pitch != 0)
    {
        long* depth2rgb = (long*)depthToRgbMap;
        for (int i = 0; i < ColorImg.rows; i++)
        {
            uchar* ptr = ColorImg.ptr<uchar>(i);  // the pointer of i-th line
            uchar* ptr2 = RgbData.ptr<uchar>(i);

            uchar* pBuffer = (uchar*)(colorLockedRect.pBits) + i * colorLockedRect.Pitch;

            for (int j = 0; j < ColorImg.cols; j++)
            {
                ptr[3 * j] = pBuffer[4 * j];  // there are four bytes in each pBuffer. The first three are BGR，the forth is not used
                ptr[3 * j + 1] = pBuffer[4 * j + 1];
                ptr[3 * j + 2] = pBuffer[4 * j + 2];


                //将彩色图像对应到深度图像上 match color image with depth image
                long x = *depth2rgb++;
                long y = *depth2rgb++;
                // If out of bounds, then don't color it at all
                if (x < 0 || y - 3 < 0 || x-2 > Width || y > Height) {
                    for (int n = 0; n < 3; ++n) ptr2[3 * j + n] = 0;
                }
                else {
                    const BYTE* curr = (uchar*)(colorLockedRect.pBits) + (x-2 + Width * (y - 3)) * 4;
                    for (int n = 0; n < 3; ++n) ptr2[3 * j + n] = curr[n];
                }
            }
        }
    }
    else
    {
        cout << "Buffer length of received texture is bogus\r\n" << endl;
    }
    pColorTexture->UnlockRect(0);
    NuiImageStreamReleaseFrame(colorStreamHandle, pColorImageFrame);

    return 1;

}


int main(int argc, char* argv[])
{
    // Initialize NUI
    if (!initKinect()) return 1;

    //Initialize Opencv variables
    cv::Mat ColorImg;
    cv::Mat DepthImg;
    cv::Mat RgbData;


    ColorImg.create(Height, Width, CV_8UC3);
    DepthImg.create(Height, Width, CV_8UC1);
    RgbData.create(Height, Width, CV_8UC3);


    //Create OpenCV windows
    cv::namedWindow("colorImage", WINDOW_AUTOSIZE);
    cv::namedWindow("depthImage", WINDOW_AUTOSIZE);
    cv::namedWindow("registered color image", WINDOW_AUTOSIZE);



    while (1)
    {

        // Wait for new image
        if (WaitForSingleObject(nextColorFrameEvent, INFINITE) == 0 && WaitForSingleObject(nextDepthFrameEvent, INFINITE) == 0)
        {

            if (!getDepth(DepthImg))
                return 1;

            if (!getColor(ColorImg, RgbData))
                return 1;

        }

        // show image
        cv::imshow("colorImage", ColorImg); 
        cv::imshow("depthImage", DepthImg);
        cv::imshow("registered color image", RgbData);


        switch (waitKey(2))
        {
            // if 'q' exit
        case 'q':

            NuiShutdown();
            return 0;

            // if 's' save image and point cloud data
        case 's':

            cv::imwrite(depthImgPath, DepthImg);
            cv::imwrite(registeredRgbImgPath, RgbData);
            cv::imwrite(colorImgPath, ColorImg);
            writeTXT(depthData, RgbData,pcdPath);
            break;

        case 't':
            static int cnt = 1;
            cv::imwrite("colorImg" + to_string(cnt) + ".jpg", ColorImg);
            cout << "colorImg" + to_string(cnt++) + ".jpg" << endl;

            break;

            // if 'd', show data in certain lines
        case 'd':
       /*
            for (int i = 0; i < Width /10; i++){
                cout << depthData[3 * i] <<" "<< depthData[3 * i + 1] <<' '<< depthData[3 * i + 2] << endl;
                if (i % Width == 0)
                    cout << endl;
            }*/
            for (int i = Height / 2 - 1; i < Height / 2 + 2; i++) {
                uchar* ptr_ = DepthImg.ptr<uchar>(i);
                for (int j = 0; j <= Width; j++) {
                    cout << depthToRgbMap[i * Width * 2 + j * 2] << ' ' << depthToRgbMap[i * Width * 2 + j * 2 + 1] << ' ' << ptr_[j] << endl;

                }
            }
        }
    }

    // shut down
    NuiShutdown();


    return 0;
}



