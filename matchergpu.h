#ifndef MATCHERGPU_H
#define MATCHERGPU_H

/*------------------------------------------------------------------------------------------*\
   This file is a GPU version of Laganiere's matcher.
\*------------------------------------------------------------------------------------------*/

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

// NEW IMPORTS from OpenCV 2.4.3
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/photo/photo.hpp"
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/nonfree/gpu.hpp"

//#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <opencv2/nonfree/gpu.hpp> // for SURF

using namespace std;


class RobustGpuMatcher
{
    private:

        // pointer to the feature point detector object
        cv::Ptr<cv::FeatureDetector> detector;
        // pointer to the feature descriptor extractor object
        cv::Ptr<cv::DescriptorExtractor> extractor;

        cv::gpu::SURF_GPU surf;


        float ratio; // max ratio between 1st and 2nd NN
        bool refineF; // if true will refine the F matrix
        double confidence; // confidence level (probability)
        double distance; // min distance to epipolar

        int _image_features_1;
        int _image_features_2;

    public:

        RobustGpuMatcher() : ratio(0.65f), refineF(true), confidence(0.99), distance(3.0)
        {
            // SURF is the default feature
            detector= new cv::SurfFeatureDetector();
            extractor= new cv::SurfDescriptorExtractor();
        }

        // Set the feature detector
        void setFeatureDetector (cv::Ptr<cv::FeatureDetector>& detect)
        {
            detector= detect;
        }

        // Set descriptor extractor
        void setDescriptorExtractor (cv::Ptr<cv::DescriptorExtractor>& desc)
        {
            extractor= desc;
        }

        // Set the minimum distance to epipolar in RANSAC
        void setMinDistanceToEpipolar (double d)
        {
            distance= d;
        }

        // Set confidence level in RANSAC
        void setConfidenceLevel (double c)
        {
            confidence= c;
        }

        // Set the NN ratio
        void setRatio (float r)
        {
            ratio= r;
        }

        // if you want the F matrix to be recalculated
        void refineFundamental (bool flag)
        {
            refineF= flag;
        }

        // Clear matches for which NN ratio is > than threshold
        // return the number of removed points
        // (corresponding entries being cleared, i.e. size will be 0)
        int ratioTest (vector <vector<cv::DMatch> > & matches)
        {
            int removed=0;

            // for all matches
            for (vector <vector<cv::DMatch> > ::iterator matchIterator= matches.begin();
                 matchIterator!= matches.end(); ++matchIterator)
            {
                // if 2 NN has been identified
                if (matchIterator->size() > 1)
                {
                    // check distance ratio
                    if ((*matchIterator)[0].distance/(*matchIterator)[1].distance > ratio)
                    {
                        matchIterator->clear(); // remove match
                        removed++;
                    }

                } else { // does not have 2 neighbours

                    matchIterator->clear(); // remove match
                    removed++;
                }
            }

            return removed;
        }

        // Insert symmetrical matches in symMatches vector
        void symmetryTest (const vector<vector<cv::DMatch> > & matches1,
                           const vector<vector<cv::DMatch> > & matches2,
                           vector<cv::DMatch>& symMatches)
        {
            // for all matches image 1 -> image 2
            for (vector<vector<cv::DMatch> > ::const_iterator matchIterator1= matches1.begin(); matchIterator1!= matches1.end(); ++matchIterator1)
            {
                if (matchIterator1->size() < 2) // ignore deleted matches
                    continue;

                // for all matches image 2 -> image 1
                for (vector<vector<cv::DMatch> > ::const_iterator matchIterator2= matches2.begin(); matchIterator2!= matches2.end(); ++matchIterator2)
                {
                    if (matchIterator2->size() < 2) continue; // ignore deleted matches

                    // Match symmetry test
                    if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx  && (*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx)
                    {
                        // add symmetrical match
                        symMatches.push_back (cv::DMatch((*matchIterator1)[0].queryIdx, (*matchIterator1)[0].trainIdx, (*matchIterator1)[0].distance));
                        break; // next match in image 1 -> image 2
                    }
                }
            }
        }

        // Identify good matches using RANSAC
        // Return f matrix
        cv::Mat ransacTest (const vector<cv::DMatch>& matches,
                            const vector<cv::KeyPoint>& keypoints1,
                            const vector<cv::KeyPoint>& keypoints2,
                            vector<cv::DMatch>& outMatches)
        {
            // Convert keypoints into Point2f
            vector<cv::Point2f> points1, points2;
            for (vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it)
            {
                // Get the position of left keypoints
                float x= keypoints1[it->queryIdx].pt.x;
                float y= keypoints1[it->queryIdx].pt.y;
                points1.push_back(cv::Point2f(x,y));
                // Get the position of right keypoints
                x= keypoints2[it->trainIdx].pt.x;
                y= keypoints2[it->trainIdx].pt.y;
                points2.push_back(cv::Point2f(x,y));
            }

            // Compute F matrix using RANSAC
            vector<uchar> inliers (points1.size(), 0);
            cv::Mat f= cv::findFundamentalMat (cv::Mat (points1),cv::Mat (points2), // matching points
                                               inliers,      // match status (inlier ou outlier)
                                               CV_FM_RANSAC, // RANSAC method
                                               distance,     // distance to epipolar line
                                               confidence);  // confidence probability

            // extract the surviving (inliers) matches
            vector<uchar>::const_iterator itIn= inliers.begin();
            vector<cv::DMatch>::const_iterator itM= matches.begin();
            // for all matches
            for ( ;itIn!= inliers.end(); ++itIn, ++itM)
            {
                if (*itIn) // it is a valid match
                {
                    outMatches.push_back(*itM);
                }
            }

            cout << "Number of matched points (after cleaning): " << outMatches.size() << endl;

            if (refineF)
            {
                // The F matrix will be recomputed with all accepted matches

                // Convert keypoints into Point2f for final F computation
                points1.clear();
                points2.clear();

                for (vector<cv::DMatch>::const_iterator it= outMatches.begin(); it!= outMatches.end(); ++it)
                {
                    // Get the position of left keypoints
                    float x= keypoints1[it->queryIdx].pt.x;
                    float y= keypoints1[it->queryIdx].pt.y;
                    points1.push_back(cv::Point2f (x,y));
                    // Get the position of right keypoints
                    x= keypoints2[it->trainIdx].pt.x;
                    y= keypoints2[it->trainIdx].pt.y;
                    points2.push_back(cv::Point2f (x,y));
                }

                // Compute 8-point F from all accepted matches
                f= cv::findFundamentalMat (cv::Mat (points1),cv::Mat (points2), // matching points
                                           CV_FM_8POINT);                       // 8-point method
            }

            return f;
        }

        void detectFeatures (cv::Mat& image1, cv::Mat& image2, // input images
                             vector<cv::KeyPoint>& keypoints1, vector<cv::KeyPoint>& keypoints2)
        {
            // 1a. Detection of the SURF features
            detector->detect (image1, keypoints1);
            detector->detect (image2, keypoints2);

            _image_features_1 = keypoints1.size ();
            _image_features_2 = keypoints2.size ();
        }


        // Match feature points using symmetry test and RANSAC
        // returns fundamental matrix
        cv::Mat match (cv::Mat& image1, cv::Mat& image2, // input images
                       vector<cv::DMatch>& matches, // output matches and keypoints
                       vector<cv::KeyPoint>& keypoints1, vector<cv::KeyPoint>& keypoints2)
        {
            // 1. Extraction of SURF descriptors

            cv::Mat descriptors1, descriptors2;
            extractor->compute (image1, keypoints1, descriptors1);
            extractor->compute (image2, keypoints2, descriptors2);

            cv::gpu::GpuMat keypoints1_GPU, descriptors1_GPU;
            cv::gpu::GpuMat keypoints2_GPU, descriptors2_GPU;

            descriptors1_GPU.upload (descriptors1);
            descriptors2_GPU.upload (descriptors2);

            /*
            //cv::GpuMat image(Size(1000, 500), CV_8UC3); // GPU memory allocations are very expensive !
            cv::gpu::GpuMat img1_gpu, img2_gpu;
            cv::gpu::GpuMat keypoints1_GPU, descriptors1_GPU;
            cv::gpu::GpuMat keypoints2_GPU, descriptors2_GPU;
            //vector<float> descriptors1_CPU, descriptors2_CPU;

            surf.keypointsRatio = 0.1f;
            surf.extended = false;
            surf.hessianThreshold = 100;
            //surf.nOctaveLayers = 2;
            //surf.nOctaves = 4;

            surf.releaseMemory();
            surf.uploadKeypoints (keypoints1, keypoints1_GPU);
            img1_gpu.upload(image1);
            surf (img1_gpu, cv::gpu::GpuMat(), keypoints1_GPU, descriptors1_GPU, false);
            surf.downloadKeypoints(keypoints1_GPU, keypoints1);
            //surf.downloadDescriptors(descriptors1_GPU, descriptors1_CPU);
            cout << "GPU descriptor matrix size: " << descriptors1_GPU.rows << " by " << descriptors1_GPU.cols << endl;
            img1_gpu.release();

            surf.releaseMemory();
            surf.uploadKeypoints (keypoints2, keypoints2_GPU);
            img2_gpu.upload(image2);
            surf (img2_gpu, cv::gpu::GpuMat(), keypoints2_GPU, descriptors2_GPU, false);
            surf.downloadKeypoints(keypoints1_GPU, keypoints2);
            //surf.downloadDescriptors(descriptors2_GPU, descriptors2_CPU);
            cout << "GPU descriptor matrix size: " << descriptors2_GPU.rows << " by " << descriptors2_GPU.cols << endl;
            surf.releaseMemory();
            img2_gpu.release();
            */

            // 2. Match the two image descriptors

            cv::gpu::GpuMat trainIdx,distance,allDist;

            // Construction of the matcher
            //cv::gpu::BruteForceMatcher_GPU_base <cv::L2<float> > matcher;
            cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > matcher;
            matcher.clear();


            // from image 1 to image 2
            // based on k nearest neighbours (with k=2)
            vector<vector<cv::DMatch> > matches1;
            // matcher.knnMatch (descriptors1_GPU, descriptors2_GPU, matches1, 2);
            matcher.knnMatchSingle (descriptors1_GPU, descriptors2_GPU,trainIdx,distance,allDist,2);
            matcher.knnMatchDownload (trainIdx,distance,matches1);
            matcher.clear();

            // from image 2 to image 1
            // based on k nearest neighbours (with k=2)
            vector<vector<cv::DMatch> > matches2;
            // matcher.knnMatch (descriptors2_GPU, descriptors1_GPU, matches2, 2);
            matcher.knnMatchSingle (descriptors2_GPU, descriptors1_GPU,trainIdx,distance,allDist,2);
            matcher.knnMatchDownload (trainIdx,distance,matches2);
            matcher.clear();

            // garbage collection
            descriptors1_GPU.release();
            descriptors2_GPU.release();
            keypoints1_GPU.release();
            keypoints2_GPU.release();


            /*
            cv::Mat descriptors1, descriptors2;
            descriptors1_GPU.download (descriptors1);
            descriptors2_GPU.download (descriptors2);

            cv::BruteForceMatcher<cv::L2<float> > matcher;

            // from image 1 to image 2
            // based on k nearest neighbours (with k=2)
            vector<vector<cv::DMatch> > matches1;
            matcher.knnMatch (descriptors1, descriptors2,
                              matches1, // vector of matches (up to 2 per entry)
                              2);	    // return 2 nearest neighbours

            // from image 2 to image 1
            // based on k nearest neighbours (with k=2)
            vector<vector<cv::DMatch> > matches2;
            matcher.knnMatch (descriptors2, descriptors1,
                              matches2, // vector of matches (up to 2 per entry)
                              2);	    // return 2 nearest neighbours
            */

            cout << "Number of matched points 1->2: " << matches1.size() << endl;
            cout << "Number of matched points 2->1: " << matches2.size() << endl;



            // 3. Remove matches for which NN ratio is > than threshold

            // clean image 1 -> image 2 matches
            int removed= ratioTest (matches1);
            cout << "Number of matched points 1->2 (ratio test) : " << matches1.size()-removed << endl;
            // clean image 2 -> image 1 matches
            removed= ratioTest (matches2);
            cout << "Number of matched points 1->2 (ratio test) : " << matches2.size()-removed << endl;



            // 4. Remove non-symmetrical matches
            vector<cv::DMatch> symMatches;
            symmetryTest(matches1,matches2,symMatches);

            cout << "Number of matched points (symmetry test): " << symMatches.size() << endl;

            // 5. Validate matches using RANSAC
            cv::Mat f = ransacTest(symMatches, keypoints1, keypoints2, matches);

            // return the found fundamental matrix
            return f;
        }

        // getters

        int getNumberFeaturesImage1 () {return _image_features_1;}
        int getNumberFeaturesImage2 () {return _image_features_2;}
};

#endif // MATCHERGPU_H
