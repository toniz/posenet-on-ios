//
//  PoseParse.cpp
//  PosenetOnIOS
//
//  Created by toniz sin on 9/7/2019.
//

#include "PoseParse.hpp"
#include "math.h"
#include <iostream>


inline int sigmoid(float x){
    return int((1/(1+exp(-x)))*FLOAT_TO_INT);
}

inline int inbetween(int min, int val, int max){
    return min > val ? min : val < max ? val : max;
}

inline int distance(int y1, int x1, int y2, int x2){
    return (y2-y1)*(y2-y1)+(x2-x1)*(x2-x1);
}

// Function: Initialization parameter value
CDecodePose::CDecodePose(){
    m_keyPointName = {"nose", "leftEye", "rightEye", "leftEar", "rightEar", "leftShoulder",
        "rightShoulder", "leftElbow", "rightElbow", "leftWrist", "rightWrist",
        "leftHip", "rightHip", "leftKnee", "rightKnee", "leftAnkle", "rightAnkle"};
    
    m_childOrder  = {0, 1, 0, 2, 0, 5, 7, 5,  11, 13, 0, 6, 8,  6,  12, 14};
    m_parentOrder = {1, 3, 2, 4, 5, 7, 9, 11, 13, 15, 6, 8, 10, 12, 14, 16};
    
    m_inputHeight = 353;
    m_inputWidth  = 257;
    m_yCnt = 23;
    m_xCnt = 17;
    m_kCnt = 17;
    m_eCnt = 16;
    m_scoreArray   = new C3DArray(m_yCnt, m_xCnt, m_kCnt);
    m_shortOffsetArray = new C3DArray(m_yCnt, m_xCnt, m_kCnt*2);
    m_middleOffsetArray = new C3DArray(m_yCnt, m_xCnt, m_eCnt*4);
    
    m_pointThreshold = int(0.1 * FLOAT_TO_INT);
    m_localWindowRadius = 1;
    m_outStride = 16;
    m_nmsRadius = 30;
    m_squaredNmsRadius = m_nmsRadius*m_nmsRadius;
}

// check the point is has the highest score in the same category in local window
// intput: y, x, k, pScore
// output: true|false, pointScore
bool CDecodePose::isMaxScoreInLocalWindow(int y, int x, int k, float* pScore, int &pointScore){
    pointScore = sigmoid(pScore[m_scoreArray->getOffset(y,x,k)]);
    if(pointScore < m_pointThreshold){
        return false;
    }

    int y_start = max(y - m_localWindowRadius, 0);
    int y_end   = min(y + m_localWindowRadius, m_yCnt - 1);
    for(int y_current = y_start; y_current < y_end + 1; y_current++){
        int x_start = max(x - m_localWindowRadius, 0);
        int x_end   = min(x + m_localWindowRadius, m_xCnt - 1);
        for(int x_current = x_start; x_current < x_end + 1; x_current++){
            int score_current = sigmoid(pScore[m_scoreArray->getOffset(y,x,k)]);
            if(score_current > pointScore){
                return false;
            }
        }
    }
    
    return true;
}

// Function: Build KeyPoint Queue Order By Score Desc.
// input: pScore
// output: topQueue
bool CDecodePose::BuildTopQueue(float *pScore, map<float, vector<int>> &topQueue){
    for(int y = 0; y < m_yCnt; y++){
        for(int x = 0; x < m_xCnt; x++){
            for(int k = 0; k < m_kCnt; k++){
                int roundScore;
                if(isMaxScoreInLocalWindow(y, x, k, pScore, roundScore)){
                    topQueue[roundScore].push_back(y);
                    topQueue[roundScore].push_back(x);
                    topQueue[roundScore].push_back(k);
                }
            }
        }
    }
    return true;
}


// Function: Filter points that are similar to the same category
// input: h, w, k, result
// output: true|false
bool CDecodePose::isInResultNmsRadius(int h, int w, int k, map<int, map<int, vector<int>>> &result){
    map<int, map<int, vector<int>>>::iterator it;
    for(it = result.begin(); it != result.end(); ++it)
    {
        int dis = distance(h, w, it->second[k][0], it->second[k][1]);
        if( dis < m_squaredNmsRadius){
            return true;
        }
    }

    return false;
}

// Function: Find Next KeyPoint
// input: isForward, edge, pScore, pShortOffset, pMiddleOffset
// input: isForward => 1 forward; 0 backward
// output: keyPoints, scoreSum
bool CDecodePose::findNextKeyPoint(int isForward, int edge, float* pScore, float *pShortOffset, float *pMiddleOffset,
                                   map<int, vector<int>> &keyPoints, int &scoreSum){
    int srcK, tagK, edgeOffset;
    if(isForward == 0){
        // backward
        srcK = m_parentOrder[edge];
        tagK = m_childOrder[edge];
        edgeOffset = m_eCnt * 2;
    }else{
        // forward
        srcK = m_childOrder[edge];
        tagK = m_parentOrder[edge];
        edgeOffset = 0;
    }
    
    if(keyPoints.count(srcK) != 0 && keyPoints.count(tagK) == 0)
    {
        int srcH = keyPoints[srcK][0];
        int srcW = keyPoints[srcK][1];
        int srcY = keyPoints[srcK][2];
        int srcX = keyPoints[srcK][3];
        float tagH = srcH * 1.0 + pMiddleOffset[m_middleOffsetArray->getOffset(srcY, srcX, edgeOffset + edge)];
        float tagW = srcW * 1.0 + pMiddleOffset[m_middleOffsetArray->getOffset(srcY, srcX, edgeOffset + edge + m_eCnt)];
        int score = 0, tagY = 0, tagX = 0;
        // Recurrent offset refinement.Particularly for large person instances, the edges ofthe kinematic
        // graph connect pairs of keypoints such asRightElbowandRight-Shoulderwhich  may  be  several  hundred
        // pixels  away  in  the  image,  making  ithard to generate accurate regressions. We have successfully
        // addressed this im-portant  issue  by  recurrently  refining  the  mid-range  pairwise  offsets  using  the
        // short-range offsets, specifically:Mk,l(x)←x′+Sl(x′),wherex′=Mk,l(x),(2)as illustrated in Fig.
        // We repeat this refinement step twice in our experiments....
        for(int step = 0; step < 4; step++)
        {
            tagY = inbetween(0, int(tagH/m_outStride), m_yCnt-1);
            tagX = inbetween(0, int(tagW/m_outStride), m_xCnt-1);
            tagH = tagY * m_outStride * 1.0 + pShortOffset[m_shortOffsetArray->getOffset(tagY, tagX, tagK)];
            tagW = tagX * m_outStride * 1.0 + pShortOffset[m_shortOffsetArray->getOffset(tagY, tagX, tagK + m_kCnt)];
            //cout << tagH << "-->" << tagW  << "|" << tagY << "-->" <<  tagX << endl;
        }

        score = sigmoid(pScore[m_scoreArray->getOffset(tagY, tagX, tagK)]);
        keyPoints[tagK].push_back(int(tagH));
        keyPoints[tagK].push_back(int(tagW));
        keyPoints[tagK].push_back(tagY);
        keyPoints[tagK].push_back(tagX);
        keyPoints[tagK].push_back(tagK);
        keyPoints[tagK].push_back(score);
        keyPoints[tagK].push_back(int(tagH/m_inputHeight*FLOAT_TO_INT));
        keyPoints[tagK].push_back(int(tagW/m_inputWidth*FLOAT_TO_INT));
        scoreSum += score;
    }
    return true;
}


/* Function: fast greedy decoding
 * Paper: https://arxiv.org/pdf/1803.08225.pdf
 * Input: score, short offset, middle offset
 * output: height => result[posescore][k][0]
 *         wigth  => result[posescore][k][1]
 *         y      => result[posescore][k][2]
 *         x      => result[posescore][k][3]
 *         k      => result[posescore][k][4]
 *         score  => result[posescore][k][5]
 *         hscale => result[posescore][k][6]  hscale/INT_TO_FLOAT
 *         wscale => result[posescore][k][7]  wscale/INT_TO_FLOAT
 */
int CDecodePose::decode(float* pScore, float *pShortOffset, float *pMiddleOffset, map<int, map<int, vector<int>>> &result){
    // We  have  developed  an  extremely  fast  greedy  decodingalgorithm to group keypoints into
    // detected person instances. We first create apriority queue, shared across allKkeypoint types,
    // in which we insert the positionxiand keypoint typekof all local maxima in the Hough score mapshk(x)
    // whichhave  score  above  a  threshold  value  (set  to  0.01  in  all  reported  experiments).
    // These points serve as candidate seeds for starting a detection instance. We thenpop elements
    // out of the queue in descending score order.
    map<float, vector<int>> topQueue;
    BuildTopQueue(pScore, topQueue);

    int scoreSum = 0;
    map<float, vector<int>>::reverse_iterator it;
    for(it = topQueue.rbegin(); it != topQueue.rend(); ++it){
        int y = it->second[0]; //y
        int x = it->second[1]; //x
        int k = it->second[2]; //k
        int score = it->first; //score
        int poiH = int(y * m_outStride * 1.0 + pShortOffset[m_shortOffsetArray->getOffset(y, x, k)]);
        int poiW = int(x * m_outStride * 1.0 + pShortOffset[m_shortOffsetArray->getOffset(y, x, k + m_kCnt)]);
        poiH = inbetween(0, poiH, m_inputHeight-1);
        poiW = inbetween(0, poiW, m_inputWidth-1);
        if(isInResultNmsRadius(poiH, poiW, k, result)){
            // At each iteration, ifthe positionxiof the current candidate detection seed
            // of typekis within a diskDr(yj′,k) of the corresponding keypoint of previously
            // detected person instancesj′, then we reject it; for this we use a non-maximum
            // suppression radius ofr= 10pixels.
            continue;
        }
        
        // Otherwise, we start a new detection instancejwith thek-th keypoint atpositionyj,
        // k=xiserving as seed. We then follow the mid-range displacementvectors along the
        // edges of the kinematic person graph to greedily connect pairs(k,l) of adjacent keypoints,
        // settingyj,l=yj,k+Mk,l(yj,k).
        map<int, vector<int>> keyPoints;
        keyPoints[k].push_back(poiH);
        keyPoints[k].push_back(poiW);
        keyPoints[k].push_back(y);
        keyPoints[k].push_back(x);
        keyPoints[k].push_back(k);
        keyPoints[k].push_back(score);
        keyPoints[k].push_back(int(poiH*1.0/m_inputHeight*FLOAT_TO_INT));
        keyPoints[k].push_back(int(poiW*1.0/m_inputWidth*FLOAT_TO_INT));
        scoreSum = score;

        // Backward Find Next KeyPoints.
        // Finally Found The KeyPoints: nose(0)
        for(int edge = m_eCnt - 1; edge > -1; edge--){
            findNextKeyPoint(0, edge, pScore, pShortOffset, pMiddleOffset, keyPoints, scoreSum);
        }
        
        // This Is Not In Paper. Nose(keypoint:0) Nms Filter.
        if(isInResultNmsRadius(keyPoints[0][0], keyPoints[0][1], 0, result)){
            continue;
        }
        
        // Forward Find Next KeyPoints
        // Finally Found All KeyPoints
        for(int edge = 0; edge < m_eCnt; edge++){
            findNextKeyPoint(1, edge, pScore, pShortOffset, pMiddleOffset, keyPoints, scoreSum);
        }

        result[int(scoreSum/m_kCnt)] = keyPoints;
    }
    
    
    return 0;
}
